/**
 * HBStarTreeNode.hpp
 * 
 * This file defines the HBStarTreeNode class, which represents a node in the
 * hierarchical B*-tree (HB*-tree) for analog placement with symmetry constraints.
 */

#pragma once

#include <memory>
#include <string>
#include "ASFBStarTree.hpp"

/**
 * Enum for node types in HB*-tree
 */
enum class HBNodeType {
    MODULE,    // Regular module node
    HIERARCHY, // Hierarchy node (represents a symmetry island)
    CONTOUR    // Contour node (represents a horizontal contour segment)
};

class HBStarTreeNode {
private:
    // Node type and name
    HBNodeType type;
    std::string name;
    
    // Tree structure links
    std::shared_ptr<HBStarTreeNode> leftChild;  // Module to the right
    std::shared_ptr<HBStarTreeNode> rightChild; // Module above
    std::weak_ptr<HBStarTreeNode> parent;       // Parent node (weak to avoid circular references)
    
    // For HIERARCHY type nodes
    std::shared_ptr<ASFBStarTree> asfTree;      // ASF-B*-tree representing the symmetry island
    
    // For CONTOUR type nodes
    int contourX1;                              // Start x-coordinate of the contour segment
    int contourY1;                              // Start y-coordinate of the contour segment
    int contourX2;                              // End x-coordinate of the contour segment
    int contourY2;                              // End y-coordinate of the contour segment
    
public:
    /**
     * Constructor
     * 
     * @param type Type of the node
     * @param name Name of the node (module name, symmetry group name, or contour identifier)
     */
    HBStarTreeNode(HBNodeType type, const std::string& name);
    
    /**
     * Destructor
     */
    ~HBStarTreeNode();
    
    /**
     * Gets the node type
     * 
     * @return Node type
     */
    HBNodeType getType() const;
    
    /**
     * Gets the node name
     * 
     * @return Node name
     */
    std::string getName() const;
    
    /**
     * Gets the module name (for MODULE type nodes)
     * 
     * @return Module name
     */
    std::string getModuleName() const;
    
    /**
     * Gets the left child (module to the right)
     * 
     * @return Left child node
     */
    std::shared_ptr<HBStarTreeNode> getLeftChild() const;
    
    /**
     * Gets the right child (module above)
     * 
     * @return Right child node
     */
    std::shared_ptr<HBStarTreeNode> getRightChild() const;
    
    /**
     * Gets the parent node
     * 
     * @return Parent node
     */
    std::shared_ptr<HBStarTreeNode> getParent() const;
    
    /**
     * Sets the left child (module to the right)
     * 
     * @param node New left child node
     */
    void setLeftChild(std::shared_ptr<HBStarTreeNode> node);
    
    /**
     * Sets the right child (module above)
     * 
     * @param node New right child node
     */
    void setRightChild(std::shared_ptr<HBStarTreeNode> node);
    
    /**
     * Sets the parent node
     * 
     * @param node New parent node
     */
    void setParent(std::shared_ptr<HBStarTreeNode> node);
    
    /**
     * Gets the ASF-B*-tree (for HIERARCHY type nodes)
     * 
     * @return ASF-B*-tree representing the symmetry island
     */
    std::shared_ptr<ASFBStarTree> getASFTree() const;
    
    /**
     * Sets the ASF-B*-tree (for HIERARCHY type nodes)
     * 
     * @param tree New ASF-B*-tree
     */
    void setASFTree(std::shared_ptr<ASFBStarTree> tree);
    
    /**
     * Sets the contour segment coordinates (for CONTOUR type nodes)
     * 
     * @param x1 Start x-coordinate
     * @param y1 Start y-coordinate
     * @param x2 End x-coordinate
     * @param y2 End y-coordinate
     */
    void setContour(int x1, int y1, int x2, int y2);
    
    /**
     * Gets the contour segment coordinates (for CONTOUR type nodes)
     * 
     * @param x1 Output: Start x-coordinate
     * @param y1 Output: Start y-coordinate
     * @param x2 Output: End x-coordinate
     * @param y2 Output: End y-coordinate
     */
    void getContour(int& x1, int& y1, int& x2, int& y2) const;
    
    /**
     * Checks if this node is a leaf node (no children)
     * 
     * @return True if the node is a leaf node, false otherwise
     */
    bool isLeaf() const;
    
    /**
     * Checks if this node is the left child of its parent
     * 
     * @return True if the node is the left child, false otherwise
     */
    bool isLeftChild() const;
    
    /**
     * Checks if this node is the right child of its parent
     * 
     * @return True if the node is the right child, false otherwise
     */
    bool isRightChild() const;
    
    /**
     * Creates a deep copy of this node and its children
     * 
     * @return A new node that is a deep copy of this one
     */
    std::shared_ptr<HBStarTreeNode> clone() const;
};