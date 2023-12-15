#ifndef NCUBEDTREE_HPP_
#define NCUBEDTREE_HPP_
#include "vendor/glm/glm/glm.hpp"
#include <vector>
#include <mutex>
#include <sstream>
#include <iostream>

template<std::uint8_t N, std::uint16_t MAXT, typename T>
class CubeTree {
    public:
    typedef struct {
        glm::vec3 center;
        glm::vec3 length;
        operator std::string() {
            std::stringstream ss;
            ss << "Center: " << center.x << " -- " << center.y << " -- " << center.z << std::endl;
            ss << "Length: " << length.x << " -- " << length.y << " -- " << length.z << std::endl;
            return ss.str();
        }
    } BBox;
    CubeTree* parent;
    CubeTree* children[N * N * N];
    constexpr static std::uint16_t MaxT{MAXT};
    std::mutex mtx;
    std::vector<T> data;
    BBox box;

    CubeTree(const BBox& box) :
        parent(nullptr),
        children(nullptr),
        box(box) {}
    
    CubeTree(CubeTree* child) :
        parent(nullptr),
        children(nullptr),
        box({child->box.center.x - ((box.length.x * (N - 1))),
            child->box.center.y - ((box.length.y * (N - 1))),
            child->box.center.z - ((box.length.z * (N - 1))),
        }, (box.length * N)) {
            children[(N * N * N) - 1] = child;
        }
    
    ~CubeTree() {
        for(CubeTree child : children) {
            if(child != nullptr) {
                delete child;
                child = nullptr;
            }
        }
    }
    
    const bool inside(const glm::vec3& pos) const {
        return
        pos.x >= (box.center.x - box.lengths.x) &&
        pos.x <= (box.center.x + box.lengths.x) &&
        pos.y >= (box.center.y - box.lengths.y) &&
        pos.y <= (box.center.y + box.lengths.y) &&
        pos.z >= (box.center.z - box.lengths.z) &&
        pos.z <= (box.center.z + box.lengths.z);
    }
    
    const bool isParent() const {
        for(CubeTree* child : children)
            if(child != nullptr) return true;
        return false;
    }
    
    private:
    CubeTree* insertToChild(T* data) {
        for(std::uint16_t i{0}; i < N; i++) {
            for(std::uint16_t j{0}; j < N; j++) {
                for(std::uint16_t k{0}; k < N; k++) {
                    const std::uint16_t childIdx{(i * N * N) + (j * N) +  k};
                    CubeTree* child{children[childIdx]};
                    const BBox childBBox{
                        {
                            (box.center.x - ((box.length.x / N) * (N - i - 1))),
                            (box.center.y - ((box.length.y / N) * (N - j - 1))),
                            (box.center.z - ((box.length.z / N) * (N - k - 1)))
                        },
                        (box.length / N)
                    };
                    if(inside(childBBox.center)) {
                        if(child == nullptr)
                            child = new CubeTree(childBBox);
                        return child->insert(data);
                    }
                }
            }
        }
    }
    
    public:
    CubeTree* insert(T* data) {
        std::lock_guard<std::mutex> lock(mtx);
        if(inside(data->m_position)) {
            if(!isParent()) {
                if(this->data.size() < MAXT) {
                    this->data.emplace_back(data);
                    return this;
                } else if(this->data.size() >= MaxT) {
                    for(CubeTree* d : this->data)
                        insertToChild(d);
                    this->data.clear();
                }
            } else {
                insertToChild(data);
            }
        } else {
            if(parent == nullptr)
                parent = new CubeTree(this);
            return parent->insert(data);
        }
    }

    operator std::string() {
        std::stringstream ss;
        ss << "Children: " << std::endl;
        for(CubeTree* child : children)
            if(child != nullptr)
                ss << child->box;
        return ss.str();
    }
    
    void PrintTree() {
        std::stringstream ss;
        for(CubeTree* child : children)
            if(child != nullptr)
                ss << std::string(child->box);
        std::cout << ss.str();
    }
};
#endif