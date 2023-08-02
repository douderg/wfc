#include <wfc.hpp>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <cassert>


namespace wfc {

Constraint::Constraint(size_t total_states):
    compatibility_map_(total_states)
{
}


void Constraint::set_compatible_states(size_t state, const std::set<size_t> neighbors) {
    compatibility_map_[state] = neighbors;
}


void Constraint::enable(size_t state, std::vector<size_t>& counters) const {
    for (size_t s : compatibility_map_[state]) {
        ++counters[s];
    }
}


void Constraint::enable(size_t state, std::vector<size_t>& counters, std::set<size_t>& toggled) const {
    for (size_t s : compatibility_map_[state]) {
        if (counters[s] == 0) {
            toggled.insert(s);
        }
        ++counters[s];
    }
}


void Constraint::disable(size_t state, std::vector<size_t>& counters) const {
    for (size_t s : compatibility_map_[state]) {
        --counters[s];
    }
}


void Constraint::disable(size_t state, std::vector<size_t>& counters, std::set<size_t>& toggled) const {
    for (size_t s : compatibility_map_[state]) {
        if (--counters[s] == 0) {
            toggled.insert(s);
        }
    }
}


bool Constraint::validate() const {
    for (size_t i = 0; i < compatibility_map_.size(); ++i) {
        for (size_t s : compatibility_map_[i]) {
            auto it = compatibility_map_[s].find(i);
            if (it == compatibility_map_[s].end()) {
                return false;
            }
        }
    }
    return true;
}


Problem::Problem(size_t count, size_t number_of_states):
    nodes_(count, Node(number_of_states)),
    number_of_states_{number_of_states}
{
}


void Problem::add_constraint(const Constraint& constraint) {
    assert(constraint.validate());
    constraints_.push_back(constraint);
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
                    constraints_[link.second.constraint].enable(state, link.second.counters, next[link.first]);
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
                    constraints_[link.second.constraint].disable(state, link.second.counters, next[link.first]);
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
    assert(nodes_);
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
    select_next_cell();
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


size_t Solution::total_states() const {
    return problem_.number_of_states_;
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

            if (fixed_[w.first]) {
                auto it = actual_removed.find(solution_[w.first]);
                if (it != actual_removed.end()) {
                    return false;
                }
            }
            
            for (auto& link : node.links) {
                step.rollback_info[w.first].links.insert(link.first);
                for (size_t state : actual_removed) {
                    problem_.constraints_[link.second.constraint].disable(state, link.second.counters, next[link.first]);
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
        solution_[next.cell] = 0;
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
        }
        for (size_t link_index : p.second.links) {
            auto& link = node.links[link_index];
            const auto& constraint = problem_.constraints_[link.constraint];
            for (size_t state : p.second.disabled) {
                constraint.enable(state, link.counters);
            }
        }
        node.available_states += p.second.disabled.size();
    }
}


bool Solution::select_next_cell() {
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
        reorder_states_ = true;
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
