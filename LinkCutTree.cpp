#include <stdexcept>
#include <queue>
#include "LinkCutTree.h"

LinkCutTree::LinkCutTree(int size) {
    splay_parent = std::vector<int>(size, -1);
    path_parent = std::vector<int>(size, -1);
    splay_left = std::vector<int>(size, -1);
    splay_right = std::vector<int>(size, -1);
    edges = std::vector<std::shared_ptr<Edge>>(size, nullptr);
    max_level_edges = std::vector<std::shared_ptr<Edge>>(size, nullptr);
}

void LinkCutTree::SplayRotateLeft(int node) {
    // brings left child of node to the top

    int left_child = splay_left[node];
    int current_parent = splay_parent[node];
    if (left_child == -1) {
        throw std::runtime_error("tried to rotate left while not having a left child");
    }

    splay_left[node] = splay_right[left_child];
    splay_right[left_child] = node;

    splay_parent[left_child] = splay_parent[node];
    splay_parent[node] = left_child;

    if (splay_left[node] != -1) {
        splay_parent[splay_left[node]] = node;
    }

    if (current_parent != -1) {
        if (splay_left[current_parent] == node) {
            splay_left[current_parent] = left_child;
        } else {
            splay_right[current_parent] = left_child;
        }
    }

    Update(node);
    Update(left_child);
}

void LinkCutTree::SplayRotateRight(int node) {
    // brings right child of node to the top

    int right_child = splay_right[node];
    int current_parent = splay_parent[node];
    if (right_child == -1) {
        throw std::runtime_error("tried to rotate right while not having a right child");
    }

    splay_right[node] = splay_left[right_child];
    splay_left[right_child] = node;

    splay_parent[right_child] = splay_parent[node];
    splay_parent[node] = right_child;

    if (splay_right[node] != -1) {
        splay_parent[splay_right[node]] = node;
    }

    if (current_parent != -1) {
        if (splay_left[current_parent] == node) {
            splay_left[current_parent] = right_child;
        } else {
            splay_right[current_parent] = right_child;
        }
    }

    Update(node);
    Update(right_child);
}

void LinkCutTree::Splay(int node) {
    while (splay_parent[node] != -1) {
        int parent = splay_parent[node];
        int grandparent = splay_parent[parent];

        if (grandparent == -1) {
            if (IsLeftChild(node)) {
                SplayRotateLeft(parent);
            } else {
                SplayRotateRight(parent);
            }
        } else {
            if (IsLeftChild(parent)) {
                if (IsLeftChild(node)) {
                    SplayRotateLeft(grandparent);
                    SplayRotateLeft(parent);
                } else {
                    SplayRotateRight(parent);
                    SplayRotateLeft(grandparent);
                }
            } else {
                if (IsLeftChild(node)) {
                    SplayRotateLeft(parent);
                    SplayRotateRight(grandparent);
                } else {
                    SplayRotateRight(grandparent);
                    SplayRotateRight(parent);
                }
            }
        }
    }
}

bool LinkCutTree::IsLeftChild(int node) {
    if (splay_parent[node] == -1) {
        throw std::runtime_error("in IsLeftChild: no parent");
    }

    return splay_left[splay_parent[node]] == node;
}

void LinkCutTree::Access(int node) {
    // returns last encountered path_parent

    Splay(node);

    if (splay_right[node] != -1) {
        path_parent[splay_right[node]] = node;
        splay_parent[splay_right[node]] = -1;
        splay_right[node] = -1;
        Update(node);
    }

    while (path_parent[node] != -1) {
        int current_path_parent = path_parent[node];
        Splay(current_path_parent);

        if (splay_right[current_path_parent] != -1) {
            path_parent[splay_right[current_path_parent]] = current_path_parent;
            splay_parent[splay_right[current_path_parent]] = -1;
        }

        splay_right[current_path_parent] = node;
        splay_parent[node] = current_path_parent;
        path_parent[node] = -1;

        Update(current_path_parent);
        Splay(node);
    }
}

int LinkCutTree::Root(int node) {
    Access(node);
    while(splay_left[node] != -1) {
        node = splay_left[node];
    }
    Splay(node);

    return node;
}

void LinkCutTree::LinkToRoot(int future_child, int future_parent, const std::shared_ptr<Edge>& edge) {
    // future_child has to be the root of its represented tree

    if (Root(future_child) != future_child) {
        throw std::runtime_error("in LinkToRoot: future_child has to be the root of its tree");
    }

    if (future_child == Root(future_parent)) {
        throw std::runtime_error("in LinkToRoot: trying to link two nodes in the same tree");
    }

    Access(future_child);
    Access(future_parent);

    splay_left[future_child] = future_parent;
    splay_parent[future_parent] = future_child;
    edges[future_child] = edge;
    Update(future_child);
    Update(future_parent);
}

void LinkCutTree::Cut(int node) {
    Access(node);
    edges[node] = nullptr;
    splay_parent[splay_left[node]] = -1;
    splay_left[node] = -1;
    Update(node);
}

void LinkCutTree::Link(int first, int second, const std::shared_ptr<Edge>& edge) {
    // links nodes first and second, second becomes parent

    if (Root(first) == Root(second)) {
        throw std::runtime_error("in Link: trying to link two nodes in the same tree");
    }

    if (!(((edge->Vertices().first == first) && (edge->Vertices().second == second)) ||
            ((edge->Vertices().first == second) && (edge->Vertices().second == first)))) {
        throw std::runtime_error("in Link: edge has wrong ends");
    }

    MakeRoot(first);
    LinkToRoot(first, second, edge);
}

void LinkCutTree::MakeRoot(int node) {
    // makes node the root of its represented tree
    // works in O(depth of node)

    Access(node);

    std::queue<int> queue;
    queue.push(node);
    while (!queue.empty()) {
        int current_node = queue.front();
        queue.pop();

        int left_child = splay_left[current_node];
        splay_left[current_node] = splay_right[current_node];
        splay_right[current_node] = left_child;

        if (splay_right[current_node] != -1) {
            queue.push(splay_right[current_node]);
        }
        if (splay_left[current_node] != -1) {
            queue.push(splay_left[current_node]);
        }
    }

    ReversePath(node);
    UpdateSplaySubTree(node);
}

void LinkCutTree::ReversePath(int node) {
    // reverses edges on the path to root
    if (edges[node] != nullptr) {
        // node is not root
        ReversePath(edges[node]->AnotherVertex(node));
        edges[edges[node]->AnotherVertex(node)] = edges[node];
        edges[node] = nullptr;
    }
}

void LinkCutTree::UpdateSplaySubTree(int node) {
    if (splay_left[node] != -1) {
        UpdateSplaySubTree(splay_left[node]);
    }
    if (splay_right[node] != -1) {
        UpdateSplaySubTree(splay_right[node]);
    }
    Update(node);
}

void LinkCutTree::Update(int node) {
    // updates the max_level_edge associated to node

    max_level_edges[node] = edges[node];

    if (splay_left[node] != -1) {
        if (max_level_edges[splay_left[node]] != nullptr) {
            if (max_level_edges[node] != nullptr) {
                if (max_level_edges[splay_left[node]]->level > max_level_edges[node]->level) {
                    max_level_edges[node] = max_level_edges[splay_left[node]];
                }
            } else {
                max_level_edges[node] = max_level_edges[splay_left[node]];
            }
        }
    }
    if (splay_right[node] != -1) {
        if (max_level_edges[splay_right[node]] != nullptr) {
            if (max_level_edges[node] != nullptr) {
                if (max_level_edges[splay_right[node]]->level > max_level_edges[node]->level) {
                    max_level_edges[node] = max_level_edges[splay_right[node]];
                }
            } else {
                max_level_edges[node] = max_level_edges[splay_right[node]];
            }
        }
    }
}

std::shared_ptr<Edge> LinkCutTree::MaxLevelEdge(int first, int second) {
    // returns edge of maximal level on the path between the input vertices

    if (Root(first) != Root(second)) {
        return nullptr;
    }

    Access(first);
    Access(second);

    Splay(first);
    std::shared_ptr<Edge> first_candidate = max_level_edges[first];

    Access(first);
    Splay(second);
    std::shared_ptr<Edge> second_candidate = max_level_edges[second];

    if (first_candidate == nullptr){
        return second_candidate;
    }
    if (second_candidate == nullptr) {
        return first_candidate;
    }

    if (first_candidate->level > second_candidate->level) {
        return first_candidate;
    }
    return second_candidate;
}
