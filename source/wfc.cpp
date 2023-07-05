#include "wfc.hpp"
#include <numeric>
#include <algorithm>
#include <deque>
#include <iostream>

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
    std::vector<bool> visited(nodes_.size());
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
                    std::set<size_t> propagated;
                    for (size_t i : constraints_[link.second.constraint][state]) {
                        if (link.second.counters[i]++ == 0) {
                            propagated.insert(i);
                        }
                    }
                    if (!visited[link.first]) {
                        next[link.first].insert(propagated.begin(), propagated.end());
                    }
                }
            }
        }
        for (const auto& s : step) {
            visited[s.first] = true;
        }
        step = next;
    }
}


std::map<size_t, std::set<size_t>> Problem::disable_states(size_t cell, const std::set<size_t>& states) {
    std::map<size_t, std::set<size_t>> result;
    std::vector<bool> visited(nodes_.size());
    using step_t = std::map<size_t, std::set<size_t>>;
    step_t step;
    step.emplace(cell, states);
    visited[cell] = true;
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
                    std::set<size_t> propagated;
                    for (size_t i : constraints_[link.second.constraint][state]) {
                        if (--link.second.counters[i] == 0) {
                            propagated.insert(i);
                        }
                    }

                    if (!visited[link.first]) {
                        next[link.first].insert(propagated.begin(), propagated.end());
                    }
                }
            }
        }
        for (const auto& s : next) {
            visited[s.first] = true;
        }
        step = next;
    }
    return result;
}


std::vector<size_t> Problem::solve() const {
    Solution solution(*this);
    while (solution.next());
    std::cout << solution.complete() << "\n";
    return solution.get_value();
}


Problem::Node::Node(size_t states):
    is_available(states),
    available_states{states}
{
}


Solution::Ordering::Ordering(const std::vector<Problem::Node>& nodes):
    nodes_{nodes}
{
}


bool Solution::Ordering::operator()(size_t x, size_t y) const {
    return nodes_[x].available_states > nodes_[y].available_states;
}


Solution::Solution(const Problem& problem):
    problem_(problem),
    fixed_(problem_.nodes_.size(), false),
    solution_(problem_.nodes_.size()),
    order_(solution_.size()),
    ordering_{problem_.nodes_}
{
    std::chrono::system_clock clock;
    rng_.seed(clock.now().time_since_epoch().count());
    std::iota(order_.begin(), order_.end(), 0);
    select_next_cell();
}


bool Solution::next() {
    if (steps_.empty()) {
        return false;
    }
    auto& step = steps_.top();
    step.state = step.available.back();
    step.available.pop_back();

    auto disabled_states = collapse_state(step);
    
    solution_[step.cell] = step.state;
    size_t conflicting_node = disable_states(step, disabled_states);
    if (conflicting_node == solution_.size()) {
        fixed_[step.cell] = true;
        return select_next_cell();
    } else {
        backtrack(conflicting_node);
    }
    
    return true;
}


bool Solution::complete() {
    return order_.empty();
}


const std::vector<size_t>& Solution::get_value() const {
    return solution_;
}


size_t Solution::disable_states(Step& step, const std::set<size_t>& states) {
    std::vector<bool> visited(problem_.nodes_.size());
    using wave_t = std::map<size_t, std::set<size_t>>;
    wave_t wave;
    wave.emplace(step.cell, states);
    visited[step.cell] = true;

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
                return w.first;
            }
            
            for (auto& link : node.links) {
                bool do_not_propagate = visited[link.first];
                if (fixed_[link.first]) {
                    auto it = actual_removed.find(solution_[link.first]);
                    if (it != actual_removed.end()) {
                        return link.first;
                    }
                    do_not_propagate = true;
                }
                step.rollback_info[w.first].links.push_back(link.first);
                std::vector<size_t> propagated;
                for (size_t state : actual_removed) {
                    const auto& allowed = problem_.constraints_[link.second.constraint][state];
                    for (size_t i : allowed) {
                        if (--link.second.counters[i] == 0) {
                            propagated.push_back(i);
                        }
                    }
                }
                if (!do_not_propagate) {
                    next[link.first].insert(propagated.begin(), propagated.end());
                }
            }
            for (size_t state : actual_removed) {
                node.is_available[state] = false;
            }
            node.available_states -= actual_removed.size();
            step.rollback_info[w.first].disabled = actual_removed;
        }

        for (const auto& w : next) {
            visited[w.first] = true;
        }
        wave = next;
    }
    return solution_.size();
}


void Solution::backtrack(size_t conflicting_node) {
    while (!steps_.empty()) {
        Step& next = steps_.top();
        
        fixed_[next.cell] = false;
        if (!next.available.empty()) {
            for (const auto& p : next.rollback_info) {
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
            next.rollback_info.clear();
            return;
        }
        for (const auto& p : next.rollback_info) {
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

        order_.push_back(next.cell);
        steps_.pop();
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
        if (next.available.empty()) {
            std::cerr << "nope\n";
            return false;
        }
        std::shuffle(next.available.begin(), next.available.end(), rng_);
        steps_.push(next);
        return true;
    }

    steps_.push(next);
    backtrack(solution_.size());
    return true;
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
