#ifndef NCUBEDTREE_HPP_
#define NCUBEDTREE_HPP_

#include "vendor/glm/glm/glm.hpp"
#include <vector>
#include <mutex>
#include <sstream>
#include <iostream>
#include <memory>
#include <algorithm>

template<std::uint8_t N, std::uint16_t MaxT, typename T, typename FType = double>
class CubeTree {
public:
    // Bounding box structure to represent the space each node covers
    typedef struct BBox {
        glm::vec<3, FType, glm::defaultp> center; // Center of the bounding box
        FType length;                             // Half-length of the bounding box
    } BBox;

    CubeTree* parent;                           // Pointer to the parent node
    CubeTree* children[N][N][N];                // Pointers to child nodes
    std::mutex mtx;                             // Mutex for thread safety
    std::vector<std::shared_ptr<T>> data;       // Vector to store data objects
    BBox box;                                   // Bounding box of the node

    // Constructor to initialize a CubeTree with a bounding box and initial data
    CubeTree(const BBox& box, std::shared_ptr<T> data) :
        parent(nullptr),
        children(),
        box(box) {
        insert(data);
    }

    // Constructor to initialize a parent node with a single child node
    CubeTree(CubeTree* child) :
        parent(nullptr),
        children(),
        box({child->box.center.x - ((child->box.length * (N - 1))),
             child->box.center.y - ((child->box.length * (N - 1))),
             child->box.center.z - ((child->box.length * (N - 1))),
        }, (child->box.length * N)) {
        for (std::uint8_t i{0}; i < N; i++)
            for (std::uint8_t j{0}; i < N; j++)
                for (std::uint8_t k{0}; k < N; k++)
                    children[i][j][k] = nullptr;
        children[N - 1][N - 1][N - 1] = child;
    }

    // Destructor to clean up child nodes
    ~CubeTree() {
        for (std::uint8_t i{0}; i < N; i++) {
            for (std::uint8_t j{0}; j < N; j++) {
                for (std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child{children[i][j][k]};
                    if (child != nullptr) {
                        delete child;
                        child = nullptr;
                    }
                }
            }
        }
    }

    // Function to check if a position is inside the bounding box of this node
    const bool inside(const glm::vec<3, FType, glm::defaultp>& pos) const {
        return
            pos.x >= (box.center.x - box.length) &&
            pos.x <= (box.center.x + box.length) &&
            pos.y >= (box.center.y - box.length) &&
            pos.y <= (box.center.y + box.length) &&
            pos.z >= (box.center.z - box.length) &&
            pos.z <= (box.center.z + box.length);
    }

    // Function to check if the current node has any child nodes
    const bool isParent() const {
        for (std::uint8_t i{0}; i < N; i++)
            for (std::uint8_t j{0}; j < N; j++)
                for (std::uint8_t k{0}; k < N; k++)
                    if (children[i][j][k] != nullptr) return true;
        return false;
    }

    // Function to query all objects within a given bounding box range
    const std::vector<std::shared_ptr<T>> queryRange(const BBox& range) const {
        std::vector<std::shared_ptr<T>> result;
        if (!intersects(box, range)) {
            return result; // No intersection, return empty result
        }

        // Check objects in the current node
        for (const auto& obj : data) {
            if (insideRange(obj->m_position, range)) {
                result.push_back(obj);
            }
        }

        // Recursively check children
        for (std::uint8_t i = 0; i < N; ++i) {
            for (std::uint8_t j = 0; j < N; ++j) {
                for (std::uint8_t k = 0; k < N; ++k) {
                    if (children[i][j][k] != nullptr) {
                        auto childResults = children[i][j][k]->queryRange(range);
                        result.insert(result.end(), childResults.begin(), childResults.end());
                    }
                }
            }
        }

        return result;
    }

    const std::vector<std::shared_ptr<T>> queryAroundObject(const std::shared_ptr<T>& obj, const FType rangeLength) const {
        return queryRange({obj->m_position, rangeLength});       // Use the queryRange function to find objects within this range
    }

    // Function to update the positions of all objects in the tree
    void update() {
        std::lock_guard<std::mutex> lock(mtx); // Lock the mutex for thread safety

        // Iterate over the data vector to check positions and update if needed
        for (auto it = data.begin(); it != data.end();) {
            auto& obj = *it;
            if (obj->m_previousPosition != obj->m_position) {
                // Save object to reinsert it later
                auto objToReinsert = obj;
                it = data.erase(it); // Remove object from current node

                // Update previous position
                objToReinsert->m_previousPosition = objToReinsert->m_position;

                // Insert object in its new position
                insert(objToReinsert);
            } else {
                ++it;
            }
        }

        // Iteratively update children
        for (std::uint8_t i = 0; i < N; ++i) {
            for (std::uint8_t j = 0; j < N; ++j) {
                for (std::uint8_t k = 0; k < N; ++k) {
                    if (children[i][j][k] != nullptr) {
                        children[i][j][k]->update();
                    }
                }
            }
        }
    }

private:
    // Function to remove an object from the tree
    bool remove(std::shared_ptr<T> data) {
        std::lock_guard<std::mutex> lock(mtx); // Lock the mutex for thread safety
        if (inside(data->m_position)) {
            auto it = std::find(this->data.begin(), this->data.end(), data);
            if (it != this->data.end()) {
                this->data.erase(it);
                return true;
            }

            // Recursively check children to remove the object
            for (std::uint8_t i = 0; i < N; ++i) {
                for (std::uint8_t j = 0; j < N; ++j) {
                    for (std::uint8_t k = 0; k < N; ++k) {
                        if (children[i][j][k] != nullptr) {
                            if (children[i][j][k]->remove(data)) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    // Function to check if two bounding boxes intersect
    static bool intersects(const BBox& a, const BBox& b) {
        return (abs(a.center.x - b.center.x) * 2 < (a.length + b.length)) &&
               (abs(a.center.y - b.center.y) * 2 < (a.length + b.length)) &&
               (abs(a.center.z - b.center.z) * 2 < (a.length + b.length));
    }

    // Function to check if a position is inside a given bounding box range
    static bool insideRange(const glm::vec<3, FType, glm::defaultp>& pos, const BBox& range) {
        return pos.x >= (range.center.x - range.length) &&
               pos.x <= (range.center.x + range.length) &&
               pos.y >= (range.center.y - range.length) &&
               pos.y <= (range.center.y + range.length) &&
               pos.z >= (range.center.z - range.length) &&
               pos.z <= (range.center.z + range.length);
    }

    // Function to insert data into the appropriate child node
    void insertToChild(std::shared_ptr<T> data) {
        for (std::uint8_t i{0}; i < N; i++) {
            for (std::uint8_t j{0}; j < N; j++) {
                for (std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child{children[i][j][k]};
                    const auto childLength{box.length / N};
                    const BBox childBBox{
                        {
                            box.center.x - (childLength * (N - i - 1)),
                            box.center.y - (childLength * (N - j - 1)),
                            box.center.z - (childLength * (N - k - 1))
                        }, childLength
                    };
                    if (inside(childBBox.center)) {
                        if (child == nullptr) child = new CubeTree(childBBox, data);
                        child->insert(data);
                        return;
                    }
                }
            }
        }
    }

public:
    // Function to insert data into the tree
    CubeTree* insert(std::shared_ptr<T> data) {
        std::lock_guard<std::mutex> lock(mtx); // Lock the mutex for thread safety
        if (inside(data->m_position)) {
            if (!isParent()) {
                if (this->data.size() < MaxT) {
                    this->data.emplace_back(data);
                    return this;
                }
                else if (this->data.size() >= MaxT) {
                    for (auto& d : this->data)
                        insertToChild(d);
                    this->data.clear();
                }
            }
            else {
                insertToChild(data);
            }
        }
        else {
            if (parent == nullptr) parent = new CubeTree(this);
            return parent->insert(data);
        }
    }
};

#endif

// #ifndef NCUBEDTREE_HPP_
// #define NCUBEDTREE_HPP_
// #include "vendor/glm/glm/glm.hpp"
// #include <vector>
// #include <mutex>
// #include <sstream>
// #include <iostream>
// #include <memory>

// template<std::uint8_t N, std::uint16_t MaxT, typename T, typename FType = double>
// class CubeTree {
//     public:
//     typedef struct BBox {
//         glm::vec<3, FType, glm::defaultp> center;
//         FType length;
//     } BBox;
//     CubeTree* parent;
//     CubeTree* children[N][N][N];
//     std::mutex mtx;
//     std::vector<std::shared_ptr<T>> data;
//     BBox box;
    
//     void PrintTree() {
//         std::cout << "This: " << this << std::endl;
//         std::cout << "Box: " << std::string(box) << std::endl;
//         std::cout << "Parent: " << parent << std::endl;
//         for(std::uint8_t i{0}; i < N; i++) {
//             for(std::uint8_t j{0}; j < N; j++) {
//                 for(std::uint8_t k{0}; k < N; k++) {
//                     CubeTree* child{children[i][j][k]};
//                     std::cout << "Child Idx: " << "[" << i << "][" << j << "][" << k << "]" << std::endl;
//                     if(child != nullptr)
//                         child->PrintTree();
//                     else
//                         std::cout << nullptr << std::endl;
//                 }
//             }
//         }
//     }
    
//     CubeTree(const BBox& box, std::shared_ptr<T> data) :
//         parent(nullptr),
//         children(),
//         box(box) {
//         insert(data);
//     }
    
//     CubeTree(CubeTree* child) :
//         parent(nullptr),
//         children(),
//         box({child->box.center.x - ((child->box.length * (N - 1))),
//             child->box.center.y - ((child->box.length * (N - 1))),
//             child->box.center.z - ((child->box.length * (N - 1))),
//         }, (child->box.length * N)) {
//             for(std::uint8_t i{0}; i < N; i++)
//                 for(std::uint8_t j{0}; j < N; j++)
//                     for(std::uint8_t k{0}; k < N; k++)
//                         children[i][j][k] = nullptr;
//             children[N - 1][N - 1][N - 1] = child;
//     }
    
//     ~CubeTree() {
//         for(std::uint8_t i{0}; i < N; i++) {
//             for(std::uint8_t j{0}; j < N; j++) {
//                 for(std::uint8_t k{0}; k < N; k++) {
//                     CubeTree* child{children[i][j][k]};
//                     if(child != nullptr) {
//                         delete child;
//                         child = nullptr;
//                     }
//                 }
//             }
//         }
//     }
    
//     const bool inside(const glm::vec3& pos) const {
//         return
//         pos.x >= (box.center.x - box.length) &&
//         pos.x <= (box.center.x + box.length) &&
//         pos.y >= (box.center.y - box.length) &&
//         pos.y <= (box.center.y + box.length) &&
//         pos.z >= (box.center.z - box.length) &&
//         pos.z <= (box.center.z + box.length);
//     }
    
//     const bool isParent() const {
//         for(std::uint8_t i{0}; i < N; i++)
//             for(std::uint8_t j{0}; j < N; j++)
//                 for(std::uint8_t k{0}; k < N; k++)
//                     if(children[i][j][k] != nullptr) return true;
//         return false;
//     }
    
//     private:
//     void insertToChild(std::shared_ptr<T> data) {
//         for(std::uint8_t i{0}; i < N; i++) {
//             for(std::uint8_t j{0}; j < N; j++) {
//                 for(std::uint8_t k{0}; k < N; k++) {
//                     CubeTree* child{children[i][j][k]};
//                     const auto childLength{box.length / N};
//                     const BBox childBBox{
//                         {
//                             box.center.x - (childLength * (N - i - 1)),
//                             box.center.y - (childLength * (N - j - 1)),
//                             box.center.z - (childLength * (N - k - 1))
//                         }, childLength
//                     };
//                     if(inside(childBBox.center)) {
//                         if(child == nullptr) child = new CubeTree(childBBox, data);
//                         child->insert(data);
//                         return;
//                     }
//                 }
//             }
//         }
//     }
    
//     public:
//     CubeTree* insert(std::shared_ptr<T> data) {
//         std::lock_guard<std::mutex> lock(mtx);
//         if(inside(data->m_position)) {
//             if(!isParent()) {
//                 if(this->data.size() < MaxT) {
//                     this->data.emplace_back(data);
//                     return this;
//                 }
//                 else if(this->data.size() >= MaxT) {
//                     for(auto& d : this->data)
//                         insertToChild(d);
//                     this->data.clear();
//                 }
//             }
//             else {
//                 insertToChild(data);
//             }
//         }
//         else {
//             if(parent == nullptr) parent = new CubeTree(this);
//             return parent->insert(data);
//         }
//     }
// };

// #endif