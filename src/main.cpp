#include <iostream>
#include <string>
#include <memory>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <atomic>

#include "parser/Parser.hpp"
#include "solver/solver.hpp"
#include "data_struct/Module.hpp"
#include "data_struct/SymmetryConstraint.hpp"
#include "utils/TimeoutManager.hpp"
using namespace std;

void printUsage(const char* programName) {
    cout << "Usage: " << programName << " <input_file> <output_file> [area_ratio]" << endl;
    cout << "  input_file: Path to the input .txt file" << endl;
    cout << "  output_file: Path to the output .out file" << endl;
    cout << "  area_ratio: Optional parameter for area vs. wirelength weight ratio (default 1.0)" << endl;
}

int main(int argc, char* argv[]) {
    // Check command line arguments
    if (argc < 3 || argc > 4) {
        printUsage(argv[0]);
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];
    double areaRatio = 1.0;  // Default area weight ratio
    
    // Parse optional area ratio parameter
    if (argc == 4) {
        try {
            areaRatio = stod(argv[3]);
            if (areaRatio < 0.0) {
                cerr << "Error: Area ratio must be non-negative" << endl;
                return 1;
            }
        } catch (const exception& e) {
            cerr << "Error parsing area ratio: " << e.what() << endl;
            return 1;
        }
    }
    
    // Enforce the 5 minute limit
    auto timeoutManager = make_shared<TimeoutManager>(290); // 4 mins 50 secs
    timeoutManager->startWatchdog();
    
    try {
        // Record start time
        auto startTime = chrono::steady_clock::now();
        
        // Parse input file
        map<string, shared_ptr<Module>> modules;
        vector<shared_ptr<SymmetryGroup>> symmetryGroups;
        
        cout << "Parsing input file: " << inputFile << endl;
        if (!Parser::parseInputFile(inputFile, modules, symmetryGroups)) {
            cerr << "Error parsing input file" << endl;
            return 1;
        }
        
        // Configure and run placement solver
        PlacementSolver solver;
        
        // Load problem data
        solver.loadProblem(modules, symmetryGroups);
        
        // Configure simulated annealing parameters
        solver.setAnnealingParameters(
            1000.0,     // Initial temperature
            0.1,        // Final temperature
            0.95,       // Cooling rate
            100,        // Iterations per temperature
            1000        // No improvement limit
        );
        
        // Configure perturbation probabilities
        solver.setPerturbationProbabilities(
            0.3,        // Rotate probability
            0.3,        // Move probability
            0.3,        // Swap probability
            0.05,       // Change representative probability
            0.05        // Convert symmetry type probability
        );
        
        // Set cost function weights
        solver.setCostWeights(
            areaRatio,      // Area weight
            1.0 - areaRatio // Wirelength weight (complementary to area weight)
        );
        
        // Set random seed for reproducibility (optional)
        solver.setRandomSeed(static_cast<unsigned int>(time(nullptr)));
        
        // Set the timeout manager
        solver.setTimeoutManager(timeoutManager);
        
        // Solve the placement problem
        cout << "Solving placement problem..." << endl;
        
        bool solveSuccess = false;
        try {
            solveSuccess = solver.solve();
        }
        catch (const exception& e) {
            cout << "Exception during solving: " << e.what() << endl;
            // Continue to write the best solution we have so far
        }
        
        if (!solveSuccess && !timeoutManager->hasTimedOut()) {
            cerr << "Error solving placement problem" << endl;
            return 1;
        }
        
        // Get the final solution
        int solutionArea = solver.getSolutionArea();
        auto solutionModules = solver.getSolutionModules();
        
        // If timeout occurred but we have some partial solution, still write the output
        if (timeoutManager->hasTimedOut()) {
            cout << "Writing the best solution found before timeout..." << endl;
        }
        
        // Write output file
        cout << "Writing output file: " << outputFile << endl;
        if (!Parser::writeOutputFile(outputFile, solutionModules, solutionArea)) {
            cerr << "Error writing output file" << endl;
            return 1;
        }
        
        // Display execution time
        auto endTime = chrono::steady_clock::now();
        auto executionTime = chrono::duration_cast<chrono::seconds>(endTime - startTime).count();
        cout << "Execution time: " << executionTime << " seconds" << endl;
        cout << "Final area: " << solutionArea << endl;
        
        return 0;
    }
    catch (const exception& e) {
        cerr << "Unexpected exception: " << e.what() << endl;
        return 1;
    }
}