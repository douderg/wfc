#pragma once

#include <vector>
#include <map>
#include <set>
#include <stack>
#include <random>

namespace wfc {

class Solution;


class Constraint {
public:
    Constraint() = default;
    Constraint(size_t total_states);
    Constraint(const Constraint&) = default;
    Constraint& operator=(const Constraint&) = default;
    Constraint(Constraint&&) = default;
    Constraint& operator=(Constraint&&) = default;

    void set_compatible_states(size_t state, const std::set<size_t> neighbors);

    void enable(size_t state, std::vector<size_t>& counters) const;
    void enable(size_t state, std::vector<size_t>& counters, std::set<size_t>& toggled) const;
    void disable(size_t state, std::vector<size_t>& counters) const;
    void disable(size_t state, std::vector<size_t>& counters, std::set<size_t>& toggled) const;
    bool validate() const;
private:
    std::vector<std::set<size_t>> compatibility_map_;

};

class Problem {
    friend class Solution;
public:
    Problem() = default;

    Problem(size_t node_count, size_t number_of_states);

    Problem(const Problem&) = default;
    Problem& operator=(const Problem&) = default;

    Problem(Problem&&) = default;
    Problem& operator=(Problem&&) = default;

    void add_constraint(const Constraint& constraint);

    void link(size_t src, size_t dst, size_t constraint);

    void enable_states(size_t cell, const std::set<size_t>& states);

    void disable_states(size_t cell, const std::set<size_t>& states);

    bool solve(std::vector<size_t>& result) const;

    bool solve(std::vector<size_t>& result, unsigned int seed) const;

    template <class ReorderFunc>
    bool solve(std::vector<size_t>& result, ReorderFunc&& func) const;

private:

    struct Link {
        Link() = default;

        size_t constraint;
        std::vector<size_t> counters;
    };

    struct Node {
        Node(size_t states);

        std::vector<bool> is_available;
        size_t available_states;
        std::map<size_t, Link> links;
    };

    std::vector<Node> nodes_;
    size_t number_of_states_;
    std::vector<Constraint> constraints_;
};


class Solution {
public:
    Solution() = default;
    Solution(const Problem& problem);

    Solution(const Solution&);
    Solution(Solution&&);

    Solution& operator=(const Solution&);
    Solution& operator=(Solution&&);

    template <class RNG>
    bool next(RNG&& rng) {
        if (steps_.empty()) {
            return false;
        }

        auto& step = steps_.top();
        if (reorder_states_) {
            std::shuffle(step.available.begin(), step.available.end(), std::forward<RNG>(rng));
            reorder_states_ = false;
        }
        step.state = step.available.back();
        step.available.pop_back();

        auto disabled_states = collapse_state(step);
        solution_[step.cell] = step.state;
        fixed_[step.cell] = true;
        if (disable_states(step, disabled_states)) {
            return select_next_cell();
        }  else {
            backtrack();
        }
        
        return true;
    }

    template <class ReorderFunc>
    bool next_best(ReorderFunc&& func) {
        if (steps_.empty()) {
            return false;
        }

        auto& step = steps_.top();
        if (reorder_states_) {
            reorder_states(std::forward<ReorderFunc>(func), step.available);
            reorder_states_ = false;
        }

        if (!step.available.empty()) {
            step.state = step.available.back();
            step.available.pop_back();

            auto disabled_states = collapse_state(step);
            solution_[step.cell] = step.state;
            fixed_[step.cell] = true;
            if (disable_states(step, disabled_states)) {
                return select_next_cell();
            }
        }

        backtrack();
        return true;
    }

    bool complete() const;

    const std::vector<size_t>& get_value() const;

    size_t total_states() const;
     
private:

    template <class ReorderFunc>
    void reorder_states(ReorderFunc&& func, std::vector<size_t>& remaining_available) const {
        std::forward<ReorderFunc>(func)(*this, remaining_available);
    }

    struct Step {
        struct RollbackInfo {
            std::set<size_t> disabled;
            std::set<size_t> links;
        };
        size_t cell;
        size_t state;
        std::map<size_t, RollbackInfo> rollback_info;
        std::vector<size_t> available;
    };

    class Ordering {
    public:
        Ordering() = default;
        Ordering(const std::vector<Problem::Node>& nodes);
        Ordering(const Ordering&) = default;
        Ordering(Ordering&&) = default;

        Ordering& operator=(const Ordering&) = default;
        Ordering& operator=(Ordering&&) = default;

        bool operator()(size_t x, size_t y) const;
    private:
        const std::vector<Problem::Node>* nodes_;
    };

    bool disable_states(Step& step, const std::set<size_t>& states);

    void backtrack();

    void rollback(Step& step);

    bool select_next_cell();

    std::set<size_t> collapse_state(const Step& step);

    Problem problem_;

    std::stack<Step> steps_;
    std::vector<bool> fixed_;
    std::vector<size_t> solution_;
    std::vector<size_t> order_;
    Ordering ordering_;
    bool reorder_states_;
};


template <class ReorderFunc>
inline bool Problem::solve(std::vector<size_t>& result, ReorderFunc&& func) const {
    Solution solution(*this);
    while (solution.next_best(std::forward<ReorderFunc>(func)));
    if (solution.complete()) {
        result = solution.get_value();
        return true;
    }
    return false;
}

}
