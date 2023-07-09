#include <wfc.hpp>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <cassert>

namespace wfc {

Problem::Problem(size_t count, size_t number_of_states):
    nodes_(count, Node(number_of_states)),
    number_of_states_{number_of_states}
{
}


void Problem::add_state_constraint(const constraint_t& constraint) {
    constraints_.emplace_back(constraint);
}


void Problem::link(size_t src, size_t dst, size_t constraint) {
    Link link;
    link.constraint = constraint;
    link.counters.resize(number_of_states_);
    nodes_.at(src).links.emplace(dst, link);
}


void Problem::enable_states(size_t cell, const std::set<size_t>& states) {
    using step_t = std::map<size_t, std::set<size_t>>;
    step_t step;
    step.emplace(cell, states);
    while (!step.empty()) {
        step_t next;
        for (const auto& s : step) {
            for (size_t state : s.second) {
                if (nodes_[s.first].is_available[state]) {
                    continue;
                }
                nodes_[s.first].is_available[state] = true;
                ++nodes_[s.first].available_states;
                for (auto& link : nodes_[s.first].links) {
                    for (size_t i : constraints_[link.second.constraint][state]) {
                        if (link.second.counters[i]++ == 0) {
                            next[link.first].insert(i);
                        }
                    }
                }
            }
        }
        step = std::move(next);
    }
}


void Problem::disable_states(size_t cell, const std::set<size_t>& states) {
    using step_t = std::map<size_t, std::set<size_t>>;
    step_t step;
    step.emplace(cell, states);
    while (!step.empty()) {
        step_t next;
        
        for (const auto& s : step) {
            for (size_t state : s.second) {
                auto &node = nodes_[s.first];
                if (!node.is_available[state]) {
                    continue;
                }
                node.is_available[state] = false;
                --node.available_states;
                for (auto& link : node.links) {
                    for (size_t i : constraints_[link.second.constraint][state]) {
                        if (--link.second.counters[i] == 0) {
                            next[link.first].insert(i);
                        }
                    }
                }
            }
        }
        step = std::move(next);
    }
}


bool Problem::solve(std::vector<size_t>& result) const {
    Solution solution(*this);
    std::chrono::system_clock clock;
    std::default_random_engine rng(clock.now().time_since_epoch().count());
    while (solution.next(rng));
    if (solution.complete()) {
        result = solution.get_value();
        return true;
    }
    return false;
}


bool Problem::solve(std::vector<size_t>& result, unsigned int seed) const {
    Solution solution(*this);
    std::default_random_engine rng(seed);
    while (solution.next(rng));
    if (solution.complete()) {
        result = solution.get_value();
        return true;
    }
    return false;
}


Problem::Node::Node(size_t states):
    is_available(states, false),
    available_states{0}
{
}


Solution::Ordering::Ordering(const std::vector<Problem::Node>& nodes):
    nodes_{&nodes}
{
}


bool Solution::Ordering::operator()(size_t x, size_t y) const {
    return nodes_->operator[](x).available_states > nodes_->operator[](y).available_states;
}


Solution::Solution(const Problem& problem):
    problem_(problem),
    fixed_(problem_.nodes_.size(), false),
    solution_(problem_.nodes_.size()),
    order_(solution_.size()),
    ordering_{problem_.nodes_}
{
    std::iota(order_.begin(), order_.end(), 0);
#ifndef NDEBUG
    for (const auto& node : problem_.nodes_) {
        size_t c = 0;
        for (size_t i=0; i<problem_.number_of_states_; ++i) {
            if (node.is_available[i]) {
                ++c;
            }
        }
        assert(c == node.available_states);
    }
#endif
}

Solution::Solution(const Solution& other):
    problem_(other.problem_),
    steps_(other.steps_),
    fixed_(other.fixed_),
    solution_(other.solution_),
    order_(other.order_)
{
    ordering_ = Ordering(problem_.nodes_);
}

Solution::Solution(Solution&& other):
    problem_(std::move(other.problem_)),
    steps_(std::move(other.steps_)),
    fixed_(std::move(other.fixed_)),
    solution_(std::move(other.solution_)),
    order_(std::move(other.order_))
{
    ordering_ = Ordering(problem_.nodes_);
}

Solution& Solution::operator=(const Solution& other) {
    problem_ = other.problem_;
    steps_ = other.steps_;
    fixed_ = other.fixed_;
    solution_ = other.solution_;
    order_ = other.order_;
    ordering_ = Ordering(problem_.nodes_);
    return *this;
}

Solution& Solution::operator=(Solution&& other) {
    problem_ = std::move(other.problem_);
    steps_ = std::move(other.steps_);
    fixed_ = std::move(other.fixed_);
    solution_ = std::move(other.solution_);
    order_ = std::move(other.order_);
    ordering_ = Ordering(problem_.nodes_);
    return *this;
}


bool Solution::complete() const {
    return order_.empty();
}


const std::vector<size_t>& Solution::get_value() const {
    return solution_;
}


bool Solution::disable_states(Step& step, const std::set<size_t>& states) {
    using wave_t = std::map<size_t, std::set<size_t>>;
    wave_t wave;
    wave.emplace(step.cell, states);

    while (!wave.empty()) {
        wave_t next;
        for (const auto& w : wave) {
            auto &node = problem_.nodes_[w.first];

            std::set<size_t> actual_removed;
            for (size_t state : w.second) {
                if (node.is_available[state]) {
                    actual_removed.insert(state);
                }
            }

            if (actual_removed.size() == node.available_states) {
                return false;
            }
            
            for (auto& link : node.links) {
                if (fixed_[link.first]) {
                    auto it = actual_removed.find(solution_[link.first]);
                    if (it != actual_removed.end()) {
                        return false;
                    }
                }

                step.rollback_info[w.first].links.insert(link.first);
                for (size_t state : actual_removed) {
                    const auto& allowed = problem_.constraints_[link.second.constraint][state];
                    for (size_t i : allowed) {
                        if (--link.second.counters[i] == 0) {
                            next[link.first].insert(i);
                        }
                    }
                }
            }
            for (size_t state : actual_removed) {
                node.is_available[state] = false;
            }
            node.available_states -= actual_removed.size();
            step.rollback_info[w.first].disabled.insert(actual_removed.begin(), actual_removed.end());
        }
        wave = std::move(next);
    }
    return true;
}


void Solution::backtrack() {
    while (!steps_.empty()) {
        Step& next = steps_.top();
        fixed_[next.cell] = false;
        if (!next.available.empty()) {
            rollback(next);
            next.rollback_info.clear();
            return;
        }
        rollback(next);
        order_.push_back(next.cell);
        steps_.pop();
    }
}


void Solution::rollback(Step& step) {
    for (const auto& p : step.rollback_info) {
        auto& node = problem_.nodes_[p.first];
        for (size_t state : p.second.disabled) {
            node.is_available[state] = true;

            for (size_t link : p.second.links) {
                const auto& allowed = problem_.constraints_[node.links[link].constraint][state];
                for (size_t i : allowed) {
                    ++node.links[link].counters[state];
                }
            }
        }
        node.available_states += p.second.disabled.size();
    }
}


bool Solution::select_next_cell(bool &reorder_states) {
    if (order_.empty()) {
        return false;
    }

    std::make_heap(order_.begin(), order_.end(), ordering_);
    std::pop_heap(order_.begin(), order_.end(), ordering_);
    
    Step next;
    next.cell = order_.back();
    order_.pop_back();

    Problem::Node& node = problem_.nodes_[next.cell];
    if (node.available_states) {
        for (size_t i = 0; i < problem_.number_of_states_; ++i) {
            if (node.is_available[i]) {
                next.available.push_back(i);
            }
        }
        assert(next.available.size() == node.available_states);
        reorder_states = true;
        steps_.push(next);
        return true;
    }

    steps_.push(next);
    backtrack();
    return !steps_.empty();
}


std::set<size_t> Solution::collapse_state(const Step& step) {
    std::set<size_t> result;
    for (size_t i = 0; i < problem_.number_of_states_; ++i) {
        if (i != step.state && problem_.nodes_[step.cell].is_available[i]) {
            result.insert(i);
        }
    }
    return result;
}

}
