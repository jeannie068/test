/**
 * HBStarTreeCore.cpp
 * 
 * Core functionality of the HBStarTree class.
 * 
 * Implementation of the HBStarTree class for analog placement with symmetry constraints.
 * The HB*-tree is a hierarchical framework that can simultaneously optimize the placement
 * with both symmetry islands and non-symmetric modules.
 */

#include "HBStarTree.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <limits>
using namespace std;

/**
 * Constructor
 */
HBStarTree::HBStarTree()
    : root(nullptr),
      horizontalContour(make_shared<Contour>()),
      verticalContour(make_shared<Contour>()),
      totalArea(0),
      isPacked(false) {
    // Node maps initialized as empty
}

/**
 * Destructor
 */
HBStarTree::~HBStarTree() {
    // Clear lookup maps
    nodeMap.clear();
}

/**
 * Adds a module to the tree
 */
void HBStarTree::addModule(shared_ptr<Module> module) {
    if (!module) return;
    
    // Add the module to module map
    modules[module->getName()] = module;
}

/**
 * Adds a symmetry group to the tree
 */
void HBStarTree::addSymmetryGroup(shared_ptr<SymmetryGroup> group) {
    if (!group) return;
    
    // Add the symmetry group to list
    symmetryGroups.push_back(group);
}

/**
 * Constructs the symmetry islands for each symmetry group
 */
void HBStarTree::constructSymmetryIslands() {
    // Create an ASF-B*-tree for each symmetry group
    for (const auto& group : symmetryGroups) {
        auto asfTree = make_shared<ASFBStarTree>(group);
        
        // Add modules that belong to this symmetry group
        for (const auto& pair : group->getSymmetryPairs()) {
            if (modules.find(pair.first) != modules.end()) {
                asfTree->addModule(modules[pair.first]);
            }
            if (modules.find(pair.second) != modules.end()) {
                asfTree->addModule(modules[pair.second]);
            }
        }
        
        for (const auto& moduleName : group->getSelfSymmetric()) {
            if (modules.find(moduleName) != modules.end()) {
                asfTree->addModule(modules[moduleName]);
            }
        }
        
        // Construct the initial ASF-B*-tree
        asfTree->constructInitialTree();
        
        // Create a hierarchy node for this symmetry group
        auto hierarchyNode = make_shared<HBStarTreeNode>(HBNodeType::HIERARCHY, group->getName());
        hierarchyNode->setASFTree(asfTree);
        
        // Add the hierarchy node to our map
        symmetryGroupNodes[group->getName()] = hierarchyNode;
    }
}

/**
 * Constructs the initial tree structure
 */
void HBStarTree::constructInitialTreeStructure() {
    // Collect all non-symmetry modules
    vector<string> nonSymmetryModules;
    
    // Create a set of all modules in symmetry groups
    set<string> symmetryModules;
    for (const auto& group : symmetryGroups) {
        for (const auto& pair : group->getSymmetryPairs()) {
            symmetryModules.insert(pair.first);
            symmetryModules.insert(pair.second);
        }
        for (const auto& moduleName : group->getSelfSymmetric()) {
            symmetryModules.insert(moduleName);
        }
    }
    
    // Find non-symmetry modules
    for (const auto& pair : modules) {
        if (symmetryModules.find(pair.first) == symmetryModules.end()) {
            nonSymmetryModules.push_back(pair.first);
        }
    }
    
    // Sort non-symmetry modules by area (largest first) for better initial placement
    sort(nonSymmetryModules.begin(), nonSymmetryModules.end(), 
              [this](const string& a, const string& b) {
                  return modules[a]->getArea() > modules[b]->getArea();
              });
    
    // Create nodes for non-symmetry modules
    for (const auto& moduleName : nonSymmetryModules) {
        auto node = make_shared<HBStarTreeNode>(HBNodeType::MODULE, moduleName);
        moduleNodes[moduleName] = node;
    }
    
    // Create the initial tree - simplest approach is a left-skewed tree
    if (!symmetryGroupNodes.empty() || !moduleNodes.empty()) {
        // Start with the first symmetry group as root, if any
        if (!symmetryGroupNodes.empty()) {
            root = symmetryGroupNodes.begin()->second;
            auto current = root;
            
            // Add remaining symmetry groups
            for (auto it = next(symmetryGroupNodes.begin()); it != symmetryGroupNodes.end(); ++it) {
                current->setLeftChild(it->second);
                it->second->setParent(current);
                current = it->second;
            }
            
            // Add non-symmetry modules
            for (const auto& pair : moduleNodes) {
                current->setLeftChild(pair.second);
                pair.second->setParent(current);
                current = pair.second;
            }
        }
        // Otherwise, start with the first non-symmetry module
        else if (!moduleNodes.empty()) {
            auto it = moduleNodes.begin();
            root = it->second;
            auto current = root;
            
            // Add remaining non-symmetry modules
            for (++it; it != moduleNodes.end(); ++it) {
                current->setLeftChild(it->second);
                it->second->setParent(current);
                current = it->second;
            }
        }
    }
}

/**
 * Clears the tree
 */
void HBStarTree::clearTree() {
    root = nullptr;
    moduleNodes.clear();
    symmetryGroupNodes.clear();
    isPacked = false;
}

/**
 * Constructs an initial HB*-tree
 */
void HBStarTree::constructInitialTree() {
    // Clear any existing tree
    clearTree();
    
    // First, construct symmetry islands for each symmetry group
    constructSymmetryIslands();
    
    // Then, construct the initial tree structure
    constructInitialTreeStructure();
    
    // Register all nodes in lookup maps
    if (root) {
        registerNodeInMap(root);
    }
}

/**
 * 
 * Implementation of perturbation operations for the HBStarTree class.
 * 
 */

/**
 * Performs a rotation operation on a module
 */
bool HBStarTree::rotateModule(const string& moduleName) {
    // Check if the module exists
    auto it = modules.find(moduleName);
    if (it == modules.end()) return false;
    
    auto module = it->second;
    
    // Check if the module is in a symmetry group
    bool inSymmetryGroup = false;
    shared_ptr<SymmetryGroup> group = nullptr;
    
    for (const auto& g : symmetryGroups) {
        // Check symmetry pairs
        for (const auto& pair : g->getSymmetryPairs()) {
            if (pair.first == moduleName || pair.second == moduleName) {
                inSymmetryGroup = true;
                group = g;
                break;
            }
        }
        
        // Check self-symmetric modules
        if (!inSymmetryGroup) {
            for (const auto& name : g->getSelfSymmetric()) {
                if (name == moduleName) {
                    inSymmetryGroup = true;
                    group = g;
                    break;
                }
            }
        }
        
        if (inSymmetryGroup) break;
    }
    
    // If the module is in a symmetry group, use the ASF-B*-tree to rotate it
    if (inSymmetryGroup && group) {
        auto it = symmetryGroupNodes.find(group->getName());
        if (it == symmetryGroupNodes.end()) return false;
        
        auto hierarchyNode = it->second;
        auto asfTree = hierarchyNode->getASFTree();
        
        if (!asfTree) return false;
        
        bool success = asfTree->rotateModule(moduleName);
        
        // Mark the symmetry group for repacking
        markSubtreeForRepack(hierarchyNode);
        
        // Repack affected subtrees
        if (isPacked && success) {
            repackAffectedSubtrees();
        }
        
        return success;
    }
    
    // Otherwise, just rotate the module directly
    module->rotate();
    
    // Mark the module's node for repacking
    auto node = getModuleNode(moduleName);
    if (node) {
        markSubtreeForRepack(node);
    }
    
    // Repack affected subtrees
    if (isPacked) {
        repackAffectedSubtrees();
    }
    
    return true;
}

/**
 * Moves a node to a new position in the tree
 */
bool HBStarTree::moveNode(const string& nodeName, 
                          const string& newParentName, 
                          bool asLeftChild) {
    // Use direct lookup instead of traversing the tree
    auto node = findNode(nodeName);
    auto newParent = findNode(newParentName);
    
    if (!node || !newParent) return false;
    
    // Remove the node from its current parent
    auto oldParent = node->getParent();
    if (oldParent) {
        if (oldParent->getLeftChild() == node) {
            oldParent->setLeftChild(nullptr);
        } else if (oldParent->getRightChild() == node) {
            oldParent->setRightChild(nullptr);
        }
        
        // Mark the oldParent's subtree for repacking
        markSubtreeForRepack(oldParent);
    } else if (node == root) {
        // The node is the root - find a new root
        if (node->getLeftChild()) {
            root = node->getLeftChild();
        } else if (node->getRightChild()) {
            root = node->getRightChild();
        } else {
            // No children - the tree will be empty after removal
            root = nullptr;
        }
    }
    
    // Add the node to its new parent
    node->setParent(newParent);
    if (asLeftChild) {
        // Handle existing left child
        auto existingChild = newParent->getLeftChild();
        if (existingChild) {
            // Find a place for the existing child
            if (!node->getLeftChild()) {
                node->setLeftChild(existingChild);
                existingChild->setParent(node);
            } else if (!node->getRightChild()) {
                node->setRightChild(existingChild);
                existingChild->setParent(node);
            } else {
                // Both children slots are taken - find another place
                auto current = node->getLeftChild();
                while (current->getLeftChild()) {
                    current = current->getLeftChild();
                }
                current->setLeftChild(existingChild);
                existingChild->setParent(current);
            }
            
            // Mark the existing child's subtree for repacking
            markSubtreeForRepack(existingChild);
        }
        newParent->setLeftChild(node);
    } else {
        // Handle existing right child
        auto existingChild = newParent->getRightChild();
        if (existingChild) {
            // Find a place for the existing child
            if (!node->getLeftChild()) {
                node->setLeftChild(existingChild);
                existingChild->setParent(node);
            } else if (!node->getRightChild()) {
                node->setRightChild(existingChild);
                existingChild->setParent(node);
            } else {
                // Both children slots are taken - find another place
                auto current = node->getRightChild();
                while (current->getRightChild()) {
                    current = current->getRightChild();
                }
                current->setRightChild(existingChild);
                existingChild->setParent(current);
            }
            
            // Mark the existing child's subtree for repacking
            markSubtreeForRepack(existingChild);
        }
        newParent->setRightChild(node);
    }
    
    // Mark the newParent's subtree for repacking
    markSubtreeForRepack(newParent);
    
    // Mark the node's subtree for repacking
    markSubtreeForRepack(node);
    
    // Since the tree structure has changed, repack affected subtrees
    if (isPacked) {
        repackAffectedSubtrees();
    }
    
    return true;
}

/**
 * Swaps two nodes in the tree
 */
bool HBStarTree::swapNodes(const string& nodeName1, const string& nodeName2) {
    // Use direct lookup instead of traversing the tree
    auto node1 = findNode(nodeName1);
    auto node2 = findNode(nodeName2);
    
    if (!node1 || !node2) return false;
    
    // Mark subtrees for repacking
    markSubtreeForRepack(node1);
    markSubtreeForRepack(node2);
    
    // Get parents and positions
    auto parent1 = node1->getParent();
    auto parent2 = node2->getParent();
    
    bool isLeftChild1 = node1->isLeftChild();
    bool isLeftChild2 = node2->isLeftChild();
    
    // Special case: node2 is a child of node1
    if (node1->getLeftChild() == node2 || node1->getRightChild() == node2) {
        // Detach node2 from node1
        if (node1->getLeftChild() == node2) {
            node1->setLeftChild(nullptr);
        } else {
            node1->setRightChild(nullptr);
        }
        
        // Detach node1 from its parent
        if (parent1) {
            if (isLeftChild1) {
                parent1->setLeftChild(nullptr);
            } else {
                parent1->setRightChild(nullptr);
            }
        }
        
        // Attach node1 as child of node2 in the same position
        if (node1->getLeftChild() == node2) {
            node2->setLeftChild(node1);
        } else {
            node2->setRightChild(node1);
        }
        node1->setParent(node2);
        
        // Attach node2 to node1's old parent
        if (parent1) {
            if (isLeftChild1) {
                parent1->setLeftChild(node2);
            } else {
                parent1->setRightChild(node2);
            }
            node2->setParent(parent1);
        } else {
            // node1 was the root
            root = node2;
            node2->setParent(nullptr);
        }
    }
    // Special case: node1 is a child of node2
    else if (node2->getLeftChild() == node1 || node2->getRightChild() == node1) {
        // Detach node1 from node2
        if (node2->getLeftChild() == node1) {
            node2->setLeftChild(nullptr);
        } else {
            node2->setRightChild(nullptr);
        }
        
        // Detach node2 from its parent
        if (parent2) {
            if (isLeftChild2) {
                parent2->setLeftChild(nullptr);
            } else {
                parent2->setRightChild(nullptr);
            }
        }
        
        // Attach node2 as child of node1 in the same position
        if (node2->getLeftChild() == node1) {
            node1->setLeftChild(node2);
        } else {
            node1->setRightChild(node2);
        }
        node2->setParent(node1);
        
        // Attach node1 to node2's old parent
        if (parent2) {
            if (isLeftChild2) {
                parent2->setLeftChild(node1);
            } else {
                parent2->setRightChild(node1);
            }
            node1->setParent(parent2);
        } else {
            // node2 was the root
            root = node1;
            node1->setParent(nullptr);
        }
    }
    // General case: nodes are not directly related
    else {
        // Detach nodes from parents
        if (parent1) {
            if (isLeftChild1) {
                parent1->setLeftChild(nullptr);
            } else {
                parent1->setRightChild(nullptr);
            }
        }
        
        if (parent2) {
            if (isLeftChild2) {
                parent2->setLeftChild(nullptr);
            } else {
                parent2->setRightChild(nullptr);
            }
        }
        
        // Swap children
        auto leftChild1 = node1->getLeftChild();
        auto rightChild1 = node1->getRightChild();
        auto leftChild2 = node2->getLeftChild();
        auto rightChild2 = node2->getRightChild();
        
        // Set children for node1
        node1->setLeftChild(leftChild2);
        node1->setRightChild(rightChild2);
        if (leftChild2) leftChild2->setParent(node1);
        if (rightChild2) rightChild2->setParent(node1);
        
        // Set children for node2
        node2->setLeftChild(leftChild1);
        node2->setRightChild(rightChild1);
        if (leftChild1) leftChild1->setParent(node2);
        if (rightChild1) rightChild1->setParent(node2);
        
        // Reattach nodes to opposite parents
        if (parent1) {
            if (isLeftChild1) {
                parent1->setLeftChild(node2);
            } else {
                parent1->setRightChild(node2);
            }
            node2->setParent(parent1);
        } else {
            // node1 was the root
            root = node2;
            node2->setParent(nullptr);
        }
        
        if (parent2) {
            if (isLeftChild2) {
                parent2->setLeftChild(node1);
            } else {
                parent2->setRightChild(node1);
            }
            node1->setParent(parent2);
        } else {
            // node2 was the root
            root = node1;
            node1->setParent(nullptr);
        }
    }
    
    // Repack affected subtrees
    if (isPacked) {
        repackAffectedSubtrees();
    }
    
    return true;
}

/**
 * Converts the symmetry type of a symmetry group
 */
bool HBStarTree::convertSymmetryType(const string& symmetryGroupName) {
    // Find the symmetry group
    auto it = symmetryGroupNodes.find(symmetryGroupName);
    if (it == symmetryGroupNodes.end()) return false;
    
    auto hierarchyNode = it->second;
    auto asfTree = hierarchyNode->getASFTree();
    
    if (!asfTree) return false;
    
    // Convert the symmetry type
    bool success = asfTree->convertSymmetryType();
    
    // Mark the symmetry group node for repacking
    markSubtreeForRepack(hierarchyNode);
    
    // Repack affected subtrees
    if (success && isPacked) {
        repackAffectedSubtrees();
    }
    
    return success;
}

/**
 * Changes the representative of a symmetry pair in a symmetry group
 */
bool HBStarTree::changeRepresentative(const string& symmetryGroupName, 
                                     const string& moduleName) {
    // Find the symmetry group
    auto it = symmetryGroupNodes.find(symmetryGroupName);
    if (it == symmetryGroupNodes.end()) return false;
    
    auto hierarchyNode = it->second;
    auto asfTree = hierarchyNode->getASFTree();
    
    if (!asfTree) return false;
    
    // Change the representative
    bool success = asfTree->changeRepresentative(moduleName);
    
    // Mark the symmetry group node for repacking
    markSubtreeForRepack(hierarchyNode);
    
    // Repack affected subtrees
    if (success && isPacked) {
        repackAffectedSubtrees();
    }
    
    return success;
}

/**
 * Mark a subtree for repacking after a modification
 */
void HBStarTree::markSubtreeForRepack(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Add the node and all its ancestors to the modified set
    auto current = node;
    while (current) {
        modifiedSubtrees.insert(current);
        current = current->getParent();
    }
}

/**
 * Repack only the affected subtrees
 */
void HBStarTree::repackAffectedSubtrees() {
    if (modifiedSubtrees.empty()) return;
    
    // Find the highest modified nodes (nodes with no modified ancestors)
    vector<shared_ptr<HBStarTreeNode>> rootsToRepack;
    
    for (const auto& node : modifiedSubtrees) {
        bool isRoot = true;
        auto parent = node->getParent();
        
        while (parent) {
            if (modifiedSubtrees.find(parent) != modifiedSubtrees.end()) {
                isRoot = false;
                break;
            }
            parent = parent->getParent();
        }
        
        if (isRoot) {
            rootsToRepack.push_back(node);
        }
    }
    
    // Sort by tree depth to repack lowest nodes first
    sort(rootsToRepack.begin(), rootsToRepack.end(), 
        [](const shared_ptr<HBStarTreeNode>& a, const shared_ptr<HBStarTreeNode>& b) {
            int depthA = 0, depthB = 0;
            auto currA = a, currB = b;
            
            while (currA->getParent()) {
                depthA++;
                currA = currA->getParent();
            }
            
            while (currB->getParent()) {
                depthB++;
                currB = currB->getParent();
            }
            
            return depthA > depthB;
        });
    
    // Repack each subtree
    for (const auto& node : rootsToRepack) {
        packSubtree(node);
    }
    
    // Clear the modified set
    modifiedSubtrees.clear();
}