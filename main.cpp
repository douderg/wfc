#include <wfc.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <numeric>

bool eval(size_t w, size_t h, size_t s, size_t d, std::vector<size_t>& result) {

    std::chrono::steady_clock clock;

    auto t0 = clock.now();
    wfc::Problem problem(w * h, s);
    wfc::Problem::constraint_t constraint;
    for (size_t i = 0; i < s; ++i) {
        std::set<size_t> states;
        for (size_t j = 1; j < d; ++j) {
            if (i >= j) {
                states.insert(i - j);
            }
            if (i + j < s) {
                states.insert(i + j);
            }
        }
        constraint.push_back(states);
    }
    problem.add_state_constraint(constraint);

    
    for (size_t i = 1; i < h; ++i) {
        for (size_t j = 1; j < w; ++j) {
            size_t k = i * w + j;
            problem.link(k, k - 1, 0);
            problem.link(k - 1, k, 0);
            problem.link(k, k - w, 0);
            problem.link(k - w, k, 0);
        }
    }
    problem.link(0, 1, 0);
    problem.link(1, 0, 0);
    problem.link(0, w, 0);
    problem.link(w, 0, 0);

    auto dur = [](auto&& d)->size_t {
        return std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
    };

    auto t1 = clock.now();
    std::cout << "connections " << dur(t1 - t0) << "\n";
    
    std::vector<size_t> states_v(s);
    std::iota(states_v.begin(), states_v.end(), 0);
    std::set<size_t> states(states_v.begin(), states_v.end());
    for (size_t i = 0; i < w * h; ++i) {
        problem.enable_states(i, states);
    }

    auto t2 = clock.now();

    std::cout << "initialization " << dur(t2 - t1) << "\n";

    if (!problem.solve(result)) {
        return false;
    }
    auto t3 = clock.now();
    std::cout << "solution " << dur(t3 - t2) << "\n";
    return true;
}


int main(int argc , char** argv) {
    size_t w = 64;
    size_t h = w;
    std::vector<size_t> rgb(w * h);
    if (!eval(w, h, 156, 10, rgb)) {
        std::cerr << "no solution\n";
        return EXIT_FAILURE;
    }
    std::ofstream f("output.ppm");
    f << "P3\n" << w << " " << h << "\n255\n";
    for (size_t i = 0; i < h; ++i) {
        for (size_t j = 0; j < w; ++j) {
            size_t v = rgb[w * i + j] + 100;
            f << v << " " << v << " " << v << " ";
        }
        f << "\n";
    }
    return EXIT_SUCCESS;
}