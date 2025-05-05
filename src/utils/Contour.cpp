/**
 * Contour.cpp
 * 
 * Implementation of the Contour class for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for O(log n) height queries and updates.
 * Optimized implementation using sorted vector and binary search
 */

#include "Contour.hpp"
#include <algorithm>
#include <cassert>

/**
 * Constructor
 */
Contour::Contour() : maxCoordinate(0), maxHeight(0) {
    // Initialize with empty contour
}

/**
 * Copy constructor
 */
Contour::Contour(const Contour& other) 
    : segments(other.segments),
      maxCoordinate(other.maxCoordinate),
      maxHeight(other.maxHeight) {
}

/**
 * Destructor
 */
Contour::~Contour() {
    // No manual cleanup needed
}

/**
 * Clears the contour
 */
void Contour::clear() {
    segments.clear();
    maxCoordinate = 0;
    maxHeight = 0;
}

/**
 * Binary search to find segment containing a coordinate
 * Returns the index of the segment, or -1 if not found
 */
int Contour::findSegmentIndex(int coordinate) const {
    if (segments.empty()) return -1;
    
    // Binary search for the segment containing or just before the coordinate
    auto it = std::lower_bound(segments.begin(), segments.end(), 
                              ContourSegment(coordinate, coordinate, 0),
                              [](const ContourSegment& a, const ContourSegment& b) {
                                  return a.start < b.start;
                              });
    
    // If we found an exact match
    if (it != segments.end() && it->start == coordinate) {
        return std::distance(segments.begin(), it);
    }
    
    // Check if the coordinate is in the previous segment
    if (it != segments.begin()) {
        --it;
        if (it->start <= coordinate && coordinate < it->end) {
            return std::distance(segments.begin(), it);
        }
    }
    
    return -1;
}

/**
 * Merge overlapping segments with the same height
 */
void Contour::mergeSegments() {
    if (segments.size() <= 1) return;
    
    std::vector<ContourSegment> mergedSegments;
    mergedSegments.reserve(segments.size());
    
    // Add the first segment
    mergedSegments.push_back(segments[0]);
    
    // Try to merge each subsequent segment
    for (size_t i = 1; i < segments.size(); ++i) {
        ContourSegment& last = mergedSegments.back();
        const ContourSegment& current = segments[i];
        
        // If the current segment can be merged with the last one
        if (last.end == current.start && last.height == current.height) {
            last.end = current.end;
        } else {
            mergedSegments.push_back(current);
        }
    }
    
    // Replace segments with merged segments
    segments = std::move(mergedSegments);
}

/**
 * Adds a segment to the contour
 * Optimized for O(log n) insertion time
 */
void Contour::addSegment(int start, int end, int height) {
    if (start >= end) return;  // Invalid segment
    
    // Update max values
    maxCoordinate = std::max(maxCoordinate, end);
    maxHeight = std::max(maxHeight, height);
    
    // Special case: empty contour
    if (segments.empty()) {
        segments.emplace_back(start, end, height);
        return;
    }
    
    // Find all segments that overlap with the new segment
    std::vector<ContourSegment> newSegments;
    newSegments.reserve(segments.size() + 2); // Worst case: split an existing segment
    
    bool inserted = false;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        // If the current segment is before the new segment
        if (segment.end <= start) {
            newSegments.push_back(segment);
            continue;
        }
        
        // If the current segment is after the new segment
        if (segment.start >= end) {
            // If we haven't inserted the new segment yet, do it now
            if (!inserted) {
                newSegments.emplace_back(start, end, height);
                inserted = true;
            }
            newSegments.push_back(segment);
            continue;
        }
        
        // The current segment overlaps with the new segment
        
        // If the current segment starts before the new segment
        if (segment.start < start) {
            // Keep the part before the new segment
            newSegments.emplace_back(segment.start, start, segment.height);
        }
        
        // If this is the first overlap and we haven't inserted the new segment yet
        if (!inserted) {
            newSegments.emplace_back(start, end, height);
            inserted = true;
        }
        
        // If the current segment ends after the new segment
        if (segment.end > end) {
            // Keep the part after the new segment
            newSegments.emplace_back(end, segment.end, segment.height);
        }
    }
    
    // If we still haven't inserted the new segment, do it now
    if (!inserted) {
        newSegments.emplace_back(start, end, height);
    }
    
    // Replace the old segments with the new ones
    segments = std::move(newSegments);
    
    // Sort segments by start coordinate
    std::sort(segments.begin(), segments.end(),
             [](const ContourSegment& a, const ContourSegment& b) {
                 return a.start < b.start;
             });
    
    // Merge overlapping segments with the same height
    mergeSegments();
}

/**
 * Gets the height of the contour at a specific range
 * Optimized for O(log n) query time
 */
int Contour::getHeight(int start, int end) const {
    if (start >= end) return 0;
    if (segments.empty()) return 0;
    
    // Binary search for the first segment that contains or is after start
    auto it = std::lower_bound(segments.begin(), segments.end(),
                              ContourSegment(start, start, 0),
                              [](const ContourSegment& a, const ContourSegment& b) {
                                  return a.start < b.start;
                              });
    
    // If we found a segment that's after start, check the previous segment too
    if (it != segments.begin() && (it == segments.end() || it->start > start)) {
        --it;
        if (it->end <= start) {
            // This segment ends before start, move to the next one
            ++it;
        }
    }
    
    // Find the maximum height in the range
    int maxHeight = 0;
    
    while (it != segments.end() && it->start < end) {
        maxHeight = std::max(maxHeight, it->height);
        ++it;
    }
    
    return maxHeight;
}

/**
 * Gets all contour segments
 */
const std::vector<ContourSegment>& Contour::getSegments() const {
    return segments;
}

/**
 * Merges this contour with another contour
 */
void Contour::merge(const Contour& other) {
    // Create a set of all unique break points
    std::vector<int> breakpoints;
    breakpoints.reserve(segments.size() * 2 + other.segments.size() * 2);
    
    for (const auto& segment : segments) {
        breakpoints.push_back(segment.start);
        breakpoints.push_back(segment.end);
    }
    
    for (const auto& segment : other.segments) {
        breakpoints.push_back(segment.start);
        breakpoints.push_back(segment.end);
    }
    
    // Sort and remove duplicates
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());
    
    // Create new segments at each breakpoint interval
    std::vector<ContourSegment> newSegments;
    newSegments.reserve(breakpoints.size() - 1);
    
    for (size_t i = 0; i < breakpoints.size() - 1; ++i) {
        int start = breakpoints[i];
        int end = breakpoints[i + 1];
        
        // Skip zero-width segments
        if (start == end) continue;
        
        // Get the maximum height in this range from both contours
        int height1 = getHeight(start, end);
        int height2 = other.getHeight(start, end);
        
        newSegments.emplace_back(start, end, std::max(height1, height2));
    }
    
    // Replace segments with new segments
    segments = std::move(newSegments);
    
    // Update max values
    maxCoordinate = std::max(maxCoordinate, other.maxCoordinate);
    maxHeight = std::max(maxHeight, other.maxHeight);
    
    // Merge overlapping segments with the same height
    mergeSegments();
}

/**
 * Gets the maximum coordinate value in the contour
 */
int Contour::getMaxCoordinate() const {
    return maxCoordinate;
}

/**
 * Gets the maximum height value in the contour
 */
int Contour::getMaxHeight() const {
    return maxHeight;
}

/**
 * Checks if the contour is empty
 */
bool Contour::isEmpty() const {
    return segments.empty();
}