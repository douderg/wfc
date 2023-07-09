#pragma once

#include <vector>
#include <map>
#include <set>
#include <stack>
#include <random>

namespace wfc {

class Solution;

class Problem {
    friend class Solution;
public:

    using constraint_t = std::vector<std::set<size_t>>; // encodes compatible neighboring states for each available state

    Problem() = default;

    Problem(size_t node_count, size_t number_of_states);

    void add_state_constraint(const constraint_t& constraint);

    void link(size_t src, size_t dst, size_t constraint);

    void enable_states(size_t cell, const std::set<size_t>& states);

    void disable_states(size_t cell, const std::set<size_t>& states);

    bool solve(std::vector<size_t>& result) const;

    bool solve(std::vector<size_t>& result, unsigned int seed) const;

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
    std::vector<constraint_t> constraints_;
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
        bool reorder_states = false;
        if (!select_next_cell(reorder_states)) {
            return false;
        }
        auto& step = steps_.top();
        if (reorder_states) {
            std::shuffle(step.available.begin(), step.available.end(), std::forward<RNG>(rng));
        }
        step.state = step.available.back();
        step.available.pop_back();

        auto disabled_states = collapse_state(step);
        
        solution_[step.cell] = step.state;
        if (disable_states(step, disabled_states)) {
            fixed_[step.cell] = true;
        } else {
            backtrack();
        }
        
        return true;
    }

    bool complete() const;

    const std::vector<size_t>& get_value() const;
     
private:
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

    bool select_next_cell(bool &reorder_states);

    std::set<size_t> collapse_state(const Step& step);

    Problem problem_;

    std::stack<Step> steps_;
    std::vector<bool> fixed_;
    std::vector<size_t> solution_;
    std::vector<size_t> order_;
    Ordering ordering_;
};

}
