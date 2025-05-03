#pragma once
#include <memory>
#include "HBStarTree.hpp"

class PlacementSolver {
private:
    std::shared_ptr<HBStarTree> hbTree;
    
    // Simulated annealing parameters
    double initialTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    
public:
    PlacementSolver(std::shared_ptr<HBStarTree> hbTree);
    
    void setAnnealingParameters(double initialTemp, double coolRate, int iterations);
    void solve();
    
    // Helper methods for simulated annealing
    void perturb();
    bool accept(double deltaE, double temperature) const;
    
    // Get solution
    int getSolutionArea() const;
    std::map<std::string, std::shared_ptr<Module>> getSolutionModules() const;
};