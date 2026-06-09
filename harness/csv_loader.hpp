#ifndef HARNESS_CSV_LOADER_HPP
#define HARNESS_CSV_LOADER_HPP

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace BoschChallenge {

inline std::vector<double> loadCSVColumn(const std::string& path) {
    std::vector<double> data;
    std::ifstream file(path);
    if (!file.is_open()) {
        return data;
    }
    double val;
    while (file >> val) {
        data.push_back(val);
    }
    return data;
}

} // namespace BoschChallenge

#endif // HARNESS_CSV_LOADER_HPP
