#ifndef NCUBEDTREE_HPP_
#define NCUBEDTREE_HPP_
#include "vendor/glm/glm/glm.hpp"
#include <vector>
#include <mutex>
#include <sstream>
#include <iostream>
#include <memory>

template<std::uint8_t N, std::uint16_t MAXT, typename T, typename FType = double>
class CubeTree {
    public:
    typedef struct BBox {
        glm::vec<3, FType, glm::defaultp> center;
        FType length;
        operator std::string() {
            std::stringstream ss;
            ss << "Center: " << center.x << " " << center.y << " " << center.z << " " << "Length: " << length << std::endl;
            return ss.str();
        }
    } BBox;
    CubeTree* parent;
    CubeTree* children[N][N][N];
    constexpr static std::uint16_t MaxT{MAXT};
    std::mutex mtx;
    std::vector<std::shared_ptr<T>> data;
    BBox box;
    
    void PrintTree() {
        std::cout << "This: " << this << std::endl;
        std::cout << "Box: " << std::string(box) << std::endl;
        std::cout << "Parent: " << parent << std::endl;
        for(std::uint8_t i{0}; i < N; i++) {
            for(std::uint8_t j{0}; j < N; j++) {
                for(std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child{children[i][j][k]};
                    std::cout << "Child Idx: " << "[" << i << "][" << j << "][" << k << "]" << std::endl;
                    if(child != nullptr)
                        child->PrintTree();
                    else
                        std::cout << nullptr << std::endl;
                }
            }
        }
    }
    
    CubeTree(const BBox& box, std::shared_ptr<T> data) :
        parent(nullptr),
        children(),
        box(box) {
        insert(data);
    }
    
    CubeTree(CubeTree* child) :
        parent(nullptr),
        children(),
        box({child->box.center.x - ((child->box.length * (N - 1))),
            child->box.center.y - ((child->box.length * (N - 1))),
            child->box.center.z - ((child->box.length * (N - 1))),
        }, (child->box.length * N)) {
            for(std::uint8_t i{0}; i < N; i++)
                for(std::uint8_t j{0}; j < N; j++)
                    for(std::uint8_t k{0}; k < N; k++)
                        children[i][j][k] = nullptr;
            children[N - 1][N - 1][N - 1] = child;
    }
    
    ~CubeTree() {
        for(std::uint8_t i{0}; i < N; i++) {
            for(std::uint8_t j{0}; j < N; j++) {
                for(std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child{children[i][j][k]};
                    if(child != nullptr) {
                        delete child;
                        child = nullptr;
                    }
                }
            }
        }
    }
    
    const bool inside(const glm::vec3& pos) const {
        return
        pos.x >= (box.center.x - box.length) &&
        pos.x <= (box.center.x + box.length) &&
        pos.y >= (box.center.y - box.length) &&
        pos.y <= (box.center.y + box.length) &&
        pos.z >= (box.center.z - box.length) &&
        pos.z <= (box.center.z + box.length);
    }
    
    const bool isParent() const {
        for(std::uint8_t i{0}; i < N; i++)
            for(std::uint8_t j{0}; j < N; j++)
                for(std::uint8_t k{0}; k < N; k++)
                    if(children[i][j][k] != nullptr) return true;
        return false;
    }
    
    private:
    void insertToChild(std::shared_ptr<T> data) {
        for(std::uint8_t i{0}; i < N; i++) {
            for(std::uint8_t j{0}; j < N; j++) {
                for(std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child{children[i][j][k]};
                    const auto childLength{box.length / N};
                    const BBox childBBox{
                        {
                            box.center.x - (childLength * (N - i - 1)),
                            box.center.y - (childLength * (N - j - 1)),
                            box.center.z - (childLength * (N - k - 1))
                        }, childLength
                    };
                    if(inside(childBBox.center)) {
                        if(child == nullptr)
                            child = new CubeTree(childBBox, data);
                        child->insert(data);
                        return;
                    }
                }
            }
        }
    }
    
    public:
    CubeTree* insert(std::shared_ptr<T> data) {
        std::lock_guard<std::mutex> lock(mtx);
        if(inside(data->m_position)) {
            if(!isParent()) {
                if(this->data.size() < MAXT) {
                    this->data.emplace_back(data);
                    return this;
                } else if(this->data.size() >= MaxT) {
                    for(auto& d : this->data)
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
};
#endif