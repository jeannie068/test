#pragma once
#include <string>
#include <memory>
#include <vector>
#include <map>
#include "Module.hpp"
#include "SymmetryConstraint.hpp"

class Parser {
public:
    static bool parseInputFile(const std::string& filename, 
                              std::map<std::string, std::shared_ptr<Module>>& modules,
                              std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups);
    
    static bool writeOutputFile(const std::string& filename,
                               const std::map<std::string, std::shared_ptr<Module>>& modules,
                               int totalArea);
};