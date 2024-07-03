#ifndef NCUBEDTREE_HPP_
#define NCUBEDTREE_HPP_

#include "vendor/glm/glm/glm.hpp"
#include <vector>
#include <mutex>
#include <sstream>
#include <iostream>
#include <memory>
#include <cmath>
#include <limits>
#include <queue>

template<std::uint8_t N, std::uint16_t MaxT, typename T, typename FType = double>
class CubeTree {
public:
    typedef struct BBox {
        glm::vec<3, FType, glm::defaultp> center;
        FType length;
    } BBox;

    CubeTree* parent;
    CubeTree* children[N][N][N];
    std::mutex mtx;
    std::vector<std::shared_ptr<T>> data;
    BBox box;

    // Constructor for a new tree node
    CubeTree(const BBox& box, std::shared_ptr<T> data) :
        parent(nullptr), children{nullptr}, box(box) {
        if(insert(data) != this) throw std::exception("Initial data entry not within node!");
    }

    // Constructor for creating a new parent node
    CubeTree(CubeTree* child) :
        parent(nullptr), children{nullptr}, box({
            child->box.center,
            pow(child->box.length, 2)  // Square the length to ensure child fits within the new parent
        }) {
        children[(child->box.center.x < box.center.x) ? 0 : N - 1][(child->box.center.y < box.center.y) ? 0 : N - 1][(child->box.center.z < box.center.z) ? 0 : N - 1] = child;
    }

    ~CubeTree() {
        for(std::uint8_t i{0}; i < N; i++) {
            for(std::uint8_t j{0}; j < N; j++) {
                for(std::uint8_t k{0}; k < N; k++) {
                    CubeTree* child = children[i][j][k];
                    if(child != nullptr) {
                        delete child;
                        child = nullptr;
                    }
                }
            }
        }
    }

    static const bool inside(const BBox& box, const glm::vec<3, FType, glm::defaultp>& pos) {
        const FType halfBl{box.length / 2};
        return (pos.x >= (box.center.x - halfBl) && pos.x <= (box.center.x + halfBl)) &&
            (pos.y >= (box.center.y - halfBl) && pos.y <= (box.center.y + halfBl)) &&
            (pos.z >= (box.center.z - halfBl) && pos.z <= (box.center.z + halfBl));
    }

    CubeTree* findParentNode(const std::shared_ptr<T>& entity) {
        // Start searching from the root node
        return findParentNodeRecursive(this, entity);
    }

    private:
    CubeTree* findParentNodeRecursive(CubeTree* node, const std::shared_ptr<T>& entity) {
        if (!node) return nullptr;

        // Lock the current node
        std::lock_guard<std::mutex> lock(node->mtx);

        // Check if the entity is in the current node
        for (const auto& data : node->data) {
            if (data == entity) {
                return node->parent;
            }
        }

        // Recursively search the children
        for (std::uint8_t i = 0; i < N; ++i) {
            for (std::uint8_t j = 0; j < N; ++j) {
                for (std::uint8_t k = 0; k < N; ++k) {
                    if (node->children[i][j][k] != nullptr) {
                        CubeTree* parentNode = findParentNodeRecursive(node->children[i][j][k], entity);
                        if (parentNode) return parentNode;
                    }
                }
            }
        }

        // If the entity is not found in this subtree, return nullptr
        return nullptr;
    }

    public:
    // Function to check if the current node has any children
    const bool isParent() const {
        for(std::uint8_t i{0}; i < N; i++)
            for(std::uint8_t j{0}; j < N; j++)
                for(std::uint8_t k{0}; k < N; k++)
                    if(children[i][j][k] != nullptr) return true;
        return false;
    }

    // Static function to print the tree structure
    static void printTree(const CubeTree* node, int depth = 0) {
        if(node == nullptr) {
            return;
        }

        // Indent based on depth
        std::string indent(depth * 2, ' ');

        // Print current node information
        std::cout << indent << "Node at depth " << depth << ": " << node << std::endl;
        std::cout << indent << "  Box Center: (" << node->box.center.x << ", " << node->box.center.y << ", " << node->box.center.z << ")" << std::endl;
        std::cout << indent << "  Box Length: " << node->box.length << std::endl;
        std::cout << indent << "  Data Count: " << node->data.size() << std::endl;

        // Print the positions of the data in the current node
        for(const auto& obj : node->data) {
            std::cout << indent << "  Data Name: (" << obj->m_name << ")" << std::endl;
            std::cout << indent << "  Data Prev Position: (" << obj->m_prevPosition.x << ", " << obj->m_prevPosition.y << ", " << obj->m_prevPosition.z << ")" << std::endl;
            std::cout << indent << "  Data Position: (" << obj->m_position.x << ", " << obj->m_position.y << ", " << obj->m_position.z << ")" << std::endl;
        }

        // Print children recursively
        for(std::uint8_t i = 0; i < N; ++i) {
            for(std::uint8_t j = 0; j < N; ++j) {
                for(std::uint8_t k = 0; k < N; ++k) {
                    if(node->children[i][j][k] != nullptr) {
                        std::cout << indent << "  Child [" << static_cast<int>(i) << "][" << static_cast<int>(j) << "][" << static_cast<int>(k) << "]:" << std::endl;
                        printTree(node->children[i][j][k], depth + 1);
                    }
                }
            }
        }
    }

    static void collectAndRemove(CubeTree* node, std::vector<std::shared_ptr<T>>& toReinsert, std::atomic<std::uint16_t>& activeThreads, std::mutex& collectMutex) {
        std::lock_guard<std::mutex> lock(node->mtx);
        auto it{node->data.begin()};
        while (it != node->data.end()) {
            if((*it)->m_position != (*it)->m_prevPosition) {
                toReinsert.push_back(*it);
                it = node->data.erase(it);
            } else {
                ++it;
            }
        }

        std::vector<std::future<void>> futures;
        for(std::uint8_t i = 0; i < N; i++) {
            for(std::uint8_t j = 0; j < N; j++) {
                for(std::uint8_t k = 0; k < N; k++) {
                    if(node->children[i][j][k] != nullptr) {
                        {
                            std::lock_guard<std::mutex> guard(collectMutex);
                            activeThreads++;
                        }
                        futures.push_back(std::async(std::launch::async, [node, i, j, k, &toReinsert, &activeThreads, &collectMutex]() {
                            collectAndRemove(node->children[i][j][k], toReinsert, activeThreads, collectMutex);
                            {
                                std::lock_guard<std::mutex> guard(collectMutex);
                                activeThreads--;
                            }
                        }));
                    }
                }
            }
        }

        for(auto& future : futures) {
            future.get();
        }
    }

    static CubeTree* update(const std::uint16_t& threads, CubeTree* root) {
        std::vector<std::shared_ptr<T>> toReinsert;
        std::atomic<std::uint16_t> activeThreads{0};
        std::mutex collectMutex;

        collectAndRemove(root, toReinsert, activeThreads, collectMutex);

        while (activeThreads > 0) {
            std::this_thread::yield();
        }

        std::queue<std::future<CubeTree*>> futures;
        std::mutex insertMutex;

        for(auto& data : toReinsert) {
            futures.push(std::async(std::launch::async, [&root, data, &insertMutex]() {
                std::lock_guard<std::mutex> lock(insertMutex);
                return root->insert(data);
            }));
        }

        for(auto& data : toReinsert) data->m_prevPosition = data->m_position;

        while (!futures.empty()) {
            root = futures.front().get();
            futures.pop();
        }

        // Ensure root is the topmost parent node
        while (root->parent != nullptr) {
            root = root->parent;
        }

        return root;
    }

    void forEach(const std::function<bool(std::shared_ptr<T>&)>& func) {
        applyFunctionToNode(this, func);
    }

    void forEachAsync(const std::uint16_t& threads, const std::function<bool(std::shared_ptr<T>&)>& func) {
        std::queue<std::future<void>> futures;
        std::mutex futuresMutex;
        std::atomic<std::uint16_t> activeThreads(0);

        std::function<void(CubeTree*)> applyFunctionToNodeAsync{
            [&](CubeTree* node) {
                if(node == nullptr) return;

                // Apply the function to the data in this node
                for(auto& data : node->data) {
                    if(!func(data)) return;
                }

                // Collect tasks for children
                std::vector<std::future<void>> localFutures;
                for(std::uint8_t i{0}; i < N; i++) {
                    for(std::uint8_t j{0}; j < N; j++) {
                        for(std::uint8_t k{0}; k < N; k++) {
                            if(node->children[i][j][k] != nullptr) {
                                std::lock_guard<std::mutex> lock(futuresMutex);
                                if(activeThreads < threads) {
                                    // Launch a new asynchronous task
                                    localFutures.emplace_back(std::async(std::launch::async, applyFunctionToNodeAsync, node->children[i][j][k]));
                                    activeThreads++;
                                } else {
                                    // Process synchronously if the thread limit is reached
                                    applyFunctionToNodeAsync(node->children[i][j][k]);
                                }
                            }
                        }
                    }
                }

                // Manage the futures to respect the thread limit
                {
                    std::lock_guard<std::mutex> lock(futuresMutex);
                    for(auto& future : localFutures) {
                        futures.push(std::move(future));
                    }
                }

                // Check and manage futures
                while (true) {
                    std::lock_guard<std::mutex> lock(futuresMutex);
                    if(futures.empty() || activeThreads < threads) break;

                    const auto& future{futures.front()};
                    if(future.valid()) {
                        futuresMutex.unlock(); // Unlock before waiting to avoid holding the lock while waiting
                        future.wait();
                        futuresMutex.lock(); // Re-lock after waiting
                        futures.pop();
                        activeThreads--;
                    } else {
                        futures.pop();
                    }
                }
            }
        };

        // Start with the root node
        applyFunctionToNodeAsync(this);

        // Wait for all remaining futures to complete
        while (true) {
            std::lock_guard<std::mutex> lock(futuresMutex);
            if(futures.empty()) break;

            const auto& future{futures.front()};
            if(future.valid()) {
                futuresMutex.unlock(); // Unlock before waiting to avoid holding the lock while waiting
                future.wait();
                futuresMutex.lock(); // Re-lock after waiting
            }
            futures.pop();
            activeThreads--;
        }
    }

    // Function to query entities within a range around a specified position
    void queryRange(const std::shared_ptr<T>& entity, const FType& range, std::vector<std::shared_ptr<T>>& results) {
        // Use the position of the entity as the center
        const auto& center{entity->m_position};
        // Check if the query range intersects with this node's bounding box
        if(intersects(center, range, box)) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Check the data in this node
                for(const auto& entity : data) {
                    if(glm::distance(center, entity->m_position) <= range) {
                        results.push_back(entity);
                    }
                }
            }

            // Recursively check the children
            for(std::uint8_t i{0}; i < N; i++) {
                for(std::uint8_t j{0}; j < N; j++) {
                    for(std::uint8_t k{0}; k < N; k++) {
                        if(children[i][j][k] != nullptr) {
                            children[i][j][k]->queryRange(entity, range, results);
                        }
                    }
                }
            }
        }
    }

private:
    const bool intersects(const glm::vec<3, FType, glm::defaultp>& center, FType range, const BBox& box) const {
        const glm::vec<3, FType, glm::defaultp> boxMin{box.center - glm::vec<3, FType, glm::defaultp>(box.length / 2)};
        const glm::vec<3, FType, glm::defaultp> boxMax{box.center + glm::vec<3, FType, glm::defaultp>(box.length / 2)};
        const glm::vec<3, FType, glm::defaultp> rangeMin{center - glm::vec<3, FType, glm::defaultp>(range)};
        const glm::vec<3, FType, glm::defaultp> rangeMax{center + glm::vec<3, FType, glm::defaultp>(range)};

        return (rangeMin.x <= boxMax.x && rangeMax.x >= boxMin.x) &&
               (rangeMin.y <= boxMax.y && rangeMax.y >= boxMin.y) &&
               (rangeMin.z <= boxMax.z && rangeMax.z >= boxMin.z);
    }

    static void applyFunctionToNode(CubeTree* node, const std::function<bool(std::shared_ptr<T>&)>& func) {
        // std::lock_guard<std::mutex> lock(node->mtx);

        // Apply the function to the data in this node
        for(auto& data : node->data) {
            if(!func(data)) return;
        }

        // Recursively apply the function to all children
        for(std::uint8_t i{0}; i < N; i++) {
            for(std::uint8_t j{0}; j < N; j++) {
                for(std::uint8_t k{0}; k < N; k++) {
                    if(node->children[i][j][k] != nullptr) {
                        applyFunctionToNode(node->children[i][j][k], func);
                    }
                }
            }
        }
    }

    bool remove(std::shared_ptr<T> data) {
        std::lock_guard<std::mutex> lock(mtx);
        if(inside(this->box, data->m_position)) {
            auto it{std::find(this->data.begin(), this->data.end(), data)};
            if(it != this->data.end()) {
                this->data.erase(it);
                return true;
            }

            for(std::uint8_t i{0}; i < N; i++) {
                for(std::uint8_t j{0}; j < N; j++) {
                    for(std::uint8_t k{0}; k < N; k++) {
                        if(children[i][j][k] != nullptr) {
                            if(children[i][j][k]->remove(data)) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
    
    // Function to insert data into a child node
    void insertToChild(std::shared_ptr<T> data) {
        // std::cout << "Position: (" << data->m_position.x << ", " << data->m_position.y << ", " << data->m_position.z << ")" << std::endl;
        // std::cout << "attempting to insert: " << data->m_name << " into node" << std::endl;
        for (std::uint8_t i = 0; i < N; ++i) {
            for (std::uint8_t j = 0; j < N; ++j) {
                for (std::uint8_t k = 0; k < N; ++k) {
                    // std::cout << "[" << int(N) << "][" << int(i) << "][" << int(j) << "][" << int(k) << "]\n";
                    const FType childLength{box.length / N};
                    const BBox childBBox{
                        {
                            box.center.x - (box.length / 2) + childLength * (i + 0.5f),
                            box.center.y - (box.length / 2) + childLength * (j + 0.5f),
                            box.center.z - (box.length / 2) + childLength * (k + 0.5f)
                        },
                        childLength
                    };
                    if (inside(childBBox, data->m_position)) {
                        auto& child{children[i][j][k]};
                        if (child == nullptr) {
                            child = new CubeTree(childBBox, data);
                            child->parent = this;
                        } else {
                            child->data.push_back(data);
                            // std::cout << "Pushing back: " << data->m_name << std::endl << std::endl;
                        }
                        return;
                    }
                    else {
                        // std::cout << "Data: " << data->m_name << " position was not inside bounding box of the child node! checking next!\n";
                    }
                }
            }
        }
        // std::cout << "Data: " << data->m_name << " position was not inside bounding box of the parent node!\n\n\n";
    }

public:
    // Function to insert data into the tree
    CubeTree* insert(std::shared_ptr<T> data) {
        std::lock_guard<std::mutex> lock(mtx);
        if(inside(this->box, data->m_position)) {
            if(isParent()) {
                insertToChild(data);
            }
            else {
                if(this->data.size() < MaxT) {
                    this->data.emplace_back(data);
                }
                else {
                    for(auto& d : this->data)
                        insertToChild(d);
                    this->data.clear();
                }
            }
        }
        else {
            if(parent == nullptr) parent = new CubeTree(this);
            return parent->insert(data);
        }
        return this;
    }
};

#endif