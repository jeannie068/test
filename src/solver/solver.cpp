#include "../solver/solver.hpp"
#include <iostream>
#include <ctime>
#include <algorithm>
#include <limits>
using namespace std;

/**
 * Constructor
 */
PlacementSolver::PlacementSolver()
    : hbTree(nullptr),
      initialTemperature(1000.0),
      finalTemperature(0.1),
      coolingRate(0.95),
      iterationsPerTemperature(100),
      noImprovementLimit(1000),
      probRotate(0.3),
      probMove(0.3),
      probSwap(0.3),
      probChangeRep(0.05),
      probConvertSym(0.05),
      areaWeight(1.0),
      wirelengthWeight(0.0),
      randomSeed(static_cast<unsigned int>(time(nullptr))),
      totalArea(0) {
}

PlacementSolver::~PlacementSolver() {

}

void PlacementSolver::setTimeoutManager(shared_ptr<TimeoutManager> manager) {
    timeoutManager = manager;
}

/**
 * Loads modules and symmetry constraints
 */
void PlacementSolver::loadProblem(const map<string, shared_ptr<Module>>& modules,
                                 const vector<shared_ptr<SymmetryGroup>>& symmetryGroups) {
    this->modules = modules;
    this->symmetryGroups = symmetryGroups;
    
    // Create a new HB*-tree
    hbTree = make_shared<HBStarTree>();
    
    // Add modules and symmetry groups to the HB*-tree
    for (const auto& pair : modules) {
        hbTree->addModule(pair.second);
    }
    
    for (const auto& group : symmetryGroups) {
        hbTree->addSymmetryGroup(group);
    }
}

/**
 * Creates an initial placement solution
 */
void PlacementSolver::createInitialSolution() {
    // Check if modules and symmetry groups are loaded
    if (modules.empty()) {
        cerr << "Error: No modules loaded." << endl;
        return;
    }
    
    // Construct an initial HB*-tree
    hbTree->constructInitialTree();
    
    // Pack the tree to get initial coordinates
    hbTree->pack();
    
    // Output initial area
    cout << "Initial area: " << hbTree->getArea() << endl;
}

/**
 * Sets simulated annealing parameters
 */
void PlacementSolver::setAnnealingParameters(double initialTemp, double finalTemp, double coolRate, 
                                           int iterations, int noImprovementLimit) {
    initialTemperature = initialTemp;
    finalTemperature = finalTemp;
    coolingRate = coolRate;
    iterationsPerTemperature = iterations;
    this->noImprovementLimit = noImprovementLimit;
}

/**
 * Sets perturbation probabilities
 */
void PlacementSolver::setPerturbationProbabilities(double rotate, double move, double swap, 
                                                 double changeRep, double convertSym) {
    // Check if probabilities sum to 1.0
    double sum = rotate + move + swap + changeRep + convertSym;
    if (abs(sum - 1.0) > 1e-6) {
        // Normalize probabilities to sum to 1.0
        if (sum <= 0.0) {
            // Default values if all probabilities are zero or negative
            probRotate = 0.3;
            probMove = 0.3;
            probSwap = 0.3;
            probChangeRep = 0.05;
            probConvertSym = 0.05;
            return;
        }
        
        probRotate = rotate / sum;
        probMove = move / sum;
        probSwap = swap / sum;
        probChangeRep = changeRep / sum;
        probConvertSym = convertSym / sum;
    } else {
        probRotate = rotate;
        probMove = move;
        probSwap = swap;
        probChangeRep = changeRep;
        probConvertSym = convertSym;
    }
}

/**
 * Sets cost function weights
 */
void PlacementSolver::setCostWeights(double area, double wirelength) {
    areaWeight = area;
    wirelengthWeight = wirelength;
}

/**
 * Sets random seed for reproducibility
 */
void PlacementSolver::setRandomSeed(unsigned int seed) {
    randomSeed = seed;
}

/**
 * Solves the placement problem using simulated annealing
 */
bool PlacementSolver::solve() {
    // Create initial solution if not already created
    if (!hbTree || !hbTree->getRoot()) {
        createInitialSolution();
    }
    
    if (!hbTree || !hbTree->getRoot()) {
        cerr << "Error: Failed to create initial solution." << endl;
        return false;
    }
    
    cout << "Starting simulated annealing..." << endl;
    cout << "Initial temperature: " << initialTemperature << endl;
    cout << "Final temperature: " << finalTemperature << endl;
    cout << "Cooling rate: " << coolingRate << endl;
    cout << "Iterations per temperature: " << iterationsPerTemperature << endl;
    cout << "No improvement limit: " << noImprovementLimit << endl;
    
    // Create the simulated annealing solver
    SimulatedAnnealing sa(hbTree, initialTemperature, finalTemperature, coolingRate,
                         iterationsPerTemperature, noImprovementLimit);
    
    // Set perturbation probabilities
    sa.setPerturbationProbabilities(probRotate, probMove, probSwap, probChangeRep, probConvertSym);
    
    // Set cost weights
    sa.setCostWeights(areaWeight, wirelengthWeight);
    
    // Set random seed
    sa.setSeed(randomSeed);
    
    // Pass the timeout manager
    if (timeoutManager) {
        sa.setTimeoutManager(timeoutManager);
    }
    
    // Regularly check for timeout during solving
    if (timeoutManager && timeoutManager->hasTimedOut()) {
        cout << "Timeout detected before starting SA." << endl;
        return false;
    }
    
    // Run simulated annealing
    auto result = sa.run();
    if (!result) {
        cerr << "Error: Simulated annealing failed to find a solution." << endl;
        return false;
    }
    
    // Update the HB*-tree with the best solution
    hbTree = result;
    
    // Ensure the solution is packed (should already be, but just to be safe)
    hbTree->pack();
    
    // Update statistics
    totalArea = hbTree->getArea();
    
    cout << "Simulated annealing completed." << endl;
    cout << "Final area: " << totalArea << endl;
    
    // Print statistics
    auto stats = sa.getStatistics();
    cout << "Total iterations: " << stats["totalIterations"] << endl;
    cout << "Accepted moves: " << stats["acceptedMoves"] << endl;
    cout << "Rejected moves: " << stats["rejectedMoves"] << endl;
    cout << "No improvement count: " << stats["noImprovementCount"] << endl;
    
    return true;
}

/**
 * Gets the solution area
 */
int PlacementSolver::getSolutionArea() const {
    return totalArea;
}

/**
 * Gets the solution modules with their positions
 */
map<string, shared_ptr<Module>> PlacementSolver::getSolutionModules() const {
    return modules;
}

/**
 * Gets placement solution statistics
 */
map<string, int> PlacementSolver::getStatistics() const {
    map<string, int> stats;
    stats["totalArea"] = totalArea;
    return stats;
}