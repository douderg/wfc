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

    std::vector<size_t> solve() const;

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

    Solution(const Problem& problem);

    bool next();

    bool complete();

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
        Ordering(const std::vector<Problem::Node>& nodes);
        bool operator()(size_t x, size_t y) const;
    private:
        const std::vector<Problem::Node>& nodes_;
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
    std::default_random_engine rng_;
    
};

}
