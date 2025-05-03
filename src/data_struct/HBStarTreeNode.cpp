/**
 * HBStarTreeNode.cpp
 * 
 * Implementation of the HBStarTreeNode class for the hierarchical B*-tree (HB*-tree)
 * used in analog placement with symmetry constraints.
 */

#include "HBStarTreeNode.hpp"

/**
 * Constructor
 */
HBStarTreeNode::HBStarTreeNode(HBNodeType type, const std::string& name)
    : type(type), 
      name(name), 
      leftChild(nullptr), 
      rightChild(nullptr),
      asfTree(nullptr),
      contourX1(0),
      contourY1(0),
      contourX2(0),
      contourY2(0) {
    // Parent is initialized as empty weak_ptr by default
}

/**
 * Destructor
 */
HBStarTreeNode::~HBStarTreeNode() {
    // Smart pointers handle memory cleanup
    // Note: Use weak_ptr for parent to avoid circular reference issues
}

/**
 * Gets the node type
 */
HBNodeType HBStarTreeNode::getType() const {
    return type;
}

/**
 * Gets the node name
 */
std::string HBStarTreeNode::getName() const {
    return name;
}

/**
 * Gets the module name (for MODULE type nodes)
 */
std::string HBStarTreeNode::getModuleName() const {
    // For MODULE type nodes, the name is the module name
    if (type == HBNodeType::MODULE) {
        return name;
    }
    return "";
}

/**
 * Gets the left child (module to the right)
 */
std::shared_ptr<HBStarTreeNode> HBStarTreeNode::getLeftChild() const {
    return leftChild;
}

/**
 * Gets the right child (module above)
 */
std::shared_ptr<HBStarTreeNode> HBStarTreeNode::getRightChild() const {
    return rightChild;
}

/**
 * Gets the parent node
 */
std::shared_ptr<HBStarTreeNode> HBStarTreeNode::getParent() const {
    // Convert weak_ptr to shared_ptr for returning
    return parent.lock();
}

/**
 * Sets the left child (module to the right)
 */
void HBStarTreeNode::setLeftChild(std::shared_ptr<HBStarTreeNode> node) {
    leftChild = node;
}

/**
 * Sets the right child (module above)
 */
void HBStarTreeNode::setRightChild(std::shared_ptr<HBStarTreeNode> node) {
    rightChild = node;
}

/**
 * Sets the parent node
 */
void HBStarTreeNode::setParent(std::shared_ptr<HBStarTreeNode> node) {
    parent = node;  // Automatically converts to weak_ptr
}

/**
 * Gets the ASF-B*-tree (for HIERARCHY type nodes)
 */
std::shared_ptr<ASFBStarTree> HBStarTreeNode::getASFTree() const {
    if (type != HBNodeType::HIERARCHY) {
        return nullptr;
    }
    return asfTree;
}

/**
 * Sets the ASF-B*-tree (for HIERARCHY type nodes)
 */
void HBStarTreeNode::setASFTree(std::shared_ptr<ASFBStarTree> tree) {
    if (type == HBNodeType::HIERARCHY) {
        asfTree = tree;
    }
}

/**
 * Sets the contour segment coordinates (for CONTOUR type nodes)
 */
void HBStarTreeNode::setContour(int x1, int y1, int x2, int y2) {
    if (type == HBNodeType::CONTOUR) {
        contourX1 = x1;
        contourY1 = y1;
        contourX2 = x2;
        contourY2 = y2;
    }
}

/**
 * Gets the contour segment coordinates (for CONTOUR type nodes)
 */
void HBStarTreeNode::getContour(int& x1, int& y1, int& x2, int& y2) const {
    if (type == HBNodeType::CONTOUR) {
        x1 = contourX1;
        y1 = contourY1;
        x2 = contourX2;
        y2 = contourY2;
    } else {
        // Default values for non-contour nodes
        x1 = 0;
        y1 = 0;
        x2 = 0;
        y2 = 0;
    }
}

/**
 * Checks if this node is a leaf node (no children)
 */
bool HBStarTreeNode::isLeaf() const {
    return !leftChild && !rightChild;
}

/**
 * Checks if this node is the left child of its parent
 */
bool HBStarTreeNode::isLeftChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getLeftChild().get() == this;
}

/**
 * Checks if this node is the right child of its parent
 */
bool HBStarTreeNode::isRightChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getRightChild().get() == this;
}

/**
 * Creates a deep copy of this node and its children
 */
std::shared_ptr<HBStarTreeNode> HBStarTreeNode::clone() const {
    // Create a new node with the same type and name
    auto clonedNode = std::make_shared<HBStarTreeNode>(type, name);
    
    // Copy type-specific data
    if (type == HBNodeType::HIERARCHY && asfTree) {
        clonedNode->asfTree = asfTree->clone();
    } else if (type == HBNodeType::CONTOUR) {
        clonedNode->contourX1 = contourX1;
        clonedNode->contourY1 = contourY1;
        clonedNode->contourX2 = contourX2;
        clonedNode->contourY2 = contourY2;
    }
    
    // Recursively clone children
    if (leftChild) {
        auto clonedLeftChild = leftChild->clone();
        clonedNode->setLeftChild(clonedLeftChild);
        clonedLeftChild->setParent(clonedNode);
    }
    
    if (rightChild) {
        auto clonedRightChild = rightChild->clone();
        clonedNode->setRightChild(clonedRightChild);
        clonedRightChild->setParent(clonedNode);
    }
    
    return clonedNode;
}