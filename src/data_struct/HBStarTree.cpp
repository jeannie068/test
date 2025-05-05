/**
 * HBStarTree.cpp
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
#include <stack>
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

HBStarTree::~HBStarTree() {
    // Clear lookup maps
    nodeMap.clear();
}

/* Add a module to the tree */
void HBStarTree::addModule(shared_ptr<Module> module) {
    if (!module) return;
    
    // Add the module to module map
    modules[module->getName()] = module;
}

/* Add a symmetry group to the tree */
void HBStarTree::addSymmetryGroup(shared_ptr<SymmetryGroup> group) {
    if (!group) return;
    
    // Add the symmetry group to list
    symmetryGroups.push_back(group);
}

/* Constructs the symmetry islands for each symmetry group */
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

/* Constructs the initial tree structure */
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

/* Clears the tree */
void HBStarTree::clearTree() {
    root = nullptr;
    moduleNodes.clear();
    symmetryGroupNodes.clear();
    isPacked = false;
}

/* Constructs an initial HB*-tree */
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
 * Finds the nearest contour node for a given node
 */
shared_ptr<HBStarTreeNode> HBStarTree::findNearestContourNode(shared_ptr<HBStarTreeNode> node) const {
    if (!node || !root) return nullptr;
    
    // Use BFS to find the nearest contour node
    queue<shared_ptr<HBStarTreeNode>> queue;
    queue.push(root);
    
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        
        if (current->getType() == HBNodeType::CONTOUR) {
            return current;
        }
        
        if (current->getLeftChild()) queue.push(current->getLeftChild());
        if (current->getRightChild()) queue.push(current->getRightChild());
    }
    
    return nullptr;
}

/**
 * Finds the leftmost skewed child of a node
 */
shared_ptr<HBStarTreeNode> HBStarTree::findLeftmostSkewedChild(shared_ptr<HBStarTreeNode> node) const {
    if (!node) return nullptr;
    
    auto current = node;
    while (current->getLeftChild()) {
        current = current->getLeftChild();
    }
    
    return current;
}

/**
 * Updates contour nodes after changing the ASF-B*-tree of a symmetry group
 */
void HBStarTree::updateContourNodes() {
    // Process each hierarchy node
    for (const auto& pair : symmetryGroupNodes) {
        auto hierarchyNode = pair.second;
        auto asfTree = hierarchyNode->getASFTree();
        
        if (!asfTree) continue;
        
        // Get the contours of the symmetry island
        auto contours = asfTree->getContours();
        auto horizontalContour = contours.first;
        
        // Get the horizontal contour segments
        auto segments = horizontalContour->getSegments();
        
        // Clear existing contour nodes
        vector<shared_ptr<HBStarTreeNode>> existingContourNodes;
        queue<shared_ptr<HBStarTreeNode>> queue;
        
        if (hierarchyNode->getRightChild()) {
            queue.push(hierarchyNode->getRightChild());
        }
        
        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();
            
            if (current->getType() == HBNodeType::CONTOUR) {
                existingContourNodes.push_back(current);
                
                if (current->getLeftChild()) {
                    queue.push(current->getLeftChild());
                }
                if (current->getRightChild()) {
                    queue.push(current->getRightChild());
                }
            }
        }
        
        // Create new contour nodes
        vector<shared_ptr<HBStarTreeNode>> newContourNodes;
        for (size_t i = 0; i < segments.size(); ++i) {
            auto contourNode = make_shared<HBStarTreeNode>(HBNodeType::CONTOUR, 
                                                             pair.first + "_contour_" + to_string(i));
            contourNode->setContour(segments[i].start, segments[i].height, segments[i].end, segments[i].height);
            newContourNodes.push_back(contourNode);
        }
        
        // Connect contour nodes
        if (!newContourNodes.empty()) {
            // Connect the first contour node to the hierarchy node
            hierarchyNode->setRightChild(newContourNodes[0]);
            newContourNodes[0]->setParent(hierarchyNode);
            
            // Connect the rest of the contour nodes
            for (size_t i = 0; i < newContourNodes.size() - 1; ++i) {
                newContourNodes[i]->setLeftChild(newContourNodes[i + 1]);
                newContourNodes[i + 1]->setParent(newContourNodes[i]);
            }
        }
        
        // Find dangling nodes - nodes whose parents were contour nodes that no longer exist
        vector<shared_ptr<HBStarTreeNode>> danglingNodes;
        for (const auto& oldContourNode : existingContourNodes) {
            if (oldContourNode->getRightChild()) {
                danglingNodes.push_back(oldContourNode->getRightChild());
            }
        }
        
        // Reassign dangling nodes
        for (const auto& danglingNode : danglingNodes) {
            // Find the nearest contour node
            auto nearestContourNode = findNearestContourNode(danglingNode);
            
            if (nearestContourNode) {
                if (!nearestContourNode->getRightChild()) {
                    // Attach directly as right child
                    nearestContourNode->setRightChild(danglingNode);
                    danglingNode->setParent(nearestContourNode);
                } else {
                    // Find the leftmost skewed child
                    auto leftmostSkewedChild = findLeftmostSkewedChild(nearestContourNode->getRightChild());
                    
                    leftmostSkewedChild->setLeftChild(danglingNode);
                    danglingNode->setParent(leftmostSkewedChild);
                }
            }
        }
    }
}

/**
 * Handles dangling nodes after tree modifications
 */
void HBStarTree::handleDanglingNodes() {
    // Similar to updateContourNodes but more general
    // Since this is complex and depends on implementation details, I'm leaving it blank for now
}

/**
 * Validates that all symmetry islands are placed correctly
 */
bool HBStarTree::validateSymmetryIslandPlacement() const {
    // Check each symmetry group
    for (const auto& group : symmetryGroups) {
        auto it = symmetryGroupNodes.find(group->getName());
        if (it == symmetryGroupNodes.end()) continue;
        
        auto hierarchyNode = it->second;
        auto asfTree = hierarchyNode->getASFTree();
        
        if (!asfTree || !asfTree->isSymmetricFeasible()) {
            return false;
        }
    }
    
    return true;
}

/**
 * Calculates the coordinates of all modules by packing the HB*-tree
 */
bool HBStarTree::pack() {
    if (!root) return false;
    
    // If there are modified subtrees, only repack those
    if (!modifiedSubtrees.empty()) {
        repackAffectedSubtrees();
        return true;
    }
    
    // Reset contours
    horizontalContour->clear();
    verticalContour->clear();
    
    // Initialize horizontal contour with a segment at y=0
    horizontalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Initialize vertical contour with a segment at x=0
    verticalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Pack the entire tree
    packSubtree(root);
    
    // Calculate total area
    int maxX = 0, maxY = 0;
    
    // Find the maximum coordinates from all modules
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Update total area
    totalArea = maxX * maxY;
    
    // Update contour nodes
    updateContourNodes();
    
    isPacked = true;
    
    return true;
}

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
 * 
 * @param symmetryGroupName Name of the symmetry group
 * @return True if the conversion was successful, false otherwise
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
 * 
 * @param symmetryGroupName Name of the symmetry group
 * @param moduleName Name of the module in the symmetry pair
 * @return True if the change was successful, false otherwise
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

/**
 * Pack a subtree starting from the given node
 */
void HBStarTree::packSubtree(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    switch (node->getType()) {
        case HBNodeType::MODULE: {
            // Pack a regular module
            const string& moduleName = node->getModuleName();
            auto module = modules[moduleName];
            
            if (!module) return;
            
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto asfTree = node->getParent()->getASFTree();
                        if (asfTree) {
                            x = static_cast<int>(asfTree->getSymmetryAxisPosition());
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2;
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        x = 0;
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1;
                    }
                }
            }
            
            // Calculate y-coordinate using the horizontal contour
            y = horizontalContour->getHeight(x, x + module->getWidth());
            
            // Set the module's position
            module->setPosition(x, y);
            
            // Update contours
            horizontalContour->addSegment(x, x + module->getWidth(), y + module->getHeight());
            verticalContour->addSegment(y, y + module->getHeight(), x + module->getWidth());
            
            break;
        }
        case HBNodeType::HIERARCHY: {
            // Pack a symmetry island
            auto asfTree = node->getASFTree();
            if (!asfTree) return;
            
            // Pack the ASF-B*-tree
            asfTree->pack();
            
            // Get the bounding rectangle of the symmetry island
            int minX = std::numeric_limits<int>::max();
            int minY = std::numeric_limits<int>::max();
            int symMaxX = 0;
            int symMaxY = 0;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                
                minX = min(minX, module->getX());
                minY = min(minY, module->getY());
                symMaxX = max(symMaxX, module->getX() + module->getWidth());
                symMaxY = max(symMaxY, module->getY() + module->getHeight());
            }
            
            // Calculate the position for the symmetry island
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto parentAsfTree = node->getParent()->getASFTree();
                        if (parentAsfTree) {
                            x = static_cast<int>(parentAsfTree->getSymmetryAxisPosition());
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2;
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        x = 0;
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1;
                    }
                }
            }
            
            // Calculate y-coordinate using the horizontal contour
            y = horizontalContour->getHeight(x, x + (symMaxX - minX));
            
            // Shift all modules in the symmetry island
            int deltaX = x - minX;
            int deltaY = y - minY;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                module->setPosition(module->getX() + deltaX, module->getY() + deltaY);
            }
            
            // Update contours
            horizontalContour->addSegment(x, x + (symMaxX - minX), y + (symMaxY - minY));
            verticalContour->addSegment(y, y + (symMaxY - minY), x + (symMaxX - minX));
            
            break;
        }
        case HBNodeType::CONTOUR:
            // Contour nodes don't need to be packed
            break;
    }
    
    // Recursively pack children
    if (node->getLeftChild()) {
        packSubtree(node->getLeftChild());
    }
    if (node->getRightChild()) {
        packSubtree(node->getRightChild());
    }
}
/**
 * Register a node in the lookup maps
 */
void HBStarTree::registerNodeInMap(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Add to node map by name
    nodeMap[node->getName()] = node;
    
    // Recursively register children
    if (node->getLeftChild()) {
        registerNodeInMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        registerNodeInMap(node->getRightChild());
    }
}

/**
 * Unregister a node from the lookup maps
 */
void HBStarTree::unregisterNodeFromMap(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Remove from node map
    nodeMap.erase(node->getName());
    
    // Recursively unregister children
    if (node->getLeftChild()) {
        unregisterNodeFromMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        unregisterNodeFromMap(node->getRightChild());
    }
}

/**
 * Find a node by name
 */
shared_ptr<HBStarTreeNode> HBStarTree::findNode(const string& nodeName) const {
    auto it = nodeMap.find(nodeName);
    if (it != nodeMap.end()) {
        return it->second;
    }
    return nullptr;
}

/**
 * Returns the total area of the placement
 */
int HBStarTree::getArea() const {
    return totalArea;
}

/**
 * Returns the total wire length of the placement
 */
int HBStarTree::getWireLength() const {
    // Wire length calculation depends on netlist information, return 0 for now
    return 0;
}

/**
 * Gets the root node of the HB*-tree
 */
shared_ptr<HBStarTreeNode> HBStarTree::getRoot() const {
    return root;
}

/**
 * Gets all modules in the design
 */
const map<string, shared_ptr<Module>>& HBStarTree::getModules() const {
    return modules;
}

/**
 * Gets all symmetry groups in the design
 */
const vector<shared_ptr<SymmetryGroup>>& HBStarTree::getSymmetryGroups() const {
    return symmetryGroups;
}

/**
 * Gets the module node with the given name
 */
shared_ptr<HBStarTreeNode> HBStarTree::getModuleNode(const string& moduleName) const {
    auto it = moduleNodes.find(moduleName);
    if (it == moduleNodes.end()) return nullptr;
    
    return it->second;
}

/**
 * Gets the symmetry group node with the given name
 */
shared_ptr<HBStarTreeNode> HBStarTree::getSymmetryGroupNode(const string& symmetryGroupName) const {
    auto it = symmetryGroupNodes.find(symmetryGroupName);
    if (it == symmetryGroupNodes.end()) return nullptr;
    
    return it->second;
}

/**
 * Creates a deep copy of this HB*-tree
 */
shared_ptr<HBStarTree> HBStarTree::clone() const {
    auto clone = make_shared<HBStarTree>();
    
    // Copy modules
    for (const auto& pair : modules) {
        auto moduleCopy = make_shared<Module>(*pair.second);
        clone->modules[pair.first] = moduleCopy;
    }
    
    // Copy symmetry groups
    for (const auto& group : symmetryGroups) {
        // Deep copy of symmetry group
        auto groupCopy = make_shared<SymmetryGroup>(*group);
        clone->symmetryGroups.push_back(groupCopy);
    }
    
    // Reconstruct initial tree
    clone->constructInitialTree();
    
    // Copy packed state
    clone->isPacked = isPacked;
    clone->totalArea = totalArea;
    
    // Copy contours
    clone->horizontalContour = make_shared<Contour>(*horizontalContour);
    clone->verticalContour = make_shared<Contour>(*verticalContour);
    
    return clone;
}