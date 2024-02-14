#include <stdexcept>
#include <queue>
#include "LinkCutTree.h"

LinkCutTree::LinkCutTree(int size) {
    splay_parent = std::vector<int>(size, -1);
    path_parent = std::vector<int>(size, -1);
    splay_left = std::vector<int>(size, -1);
    splay_right = std::vector<int>(size, -1);
    level = std::vector<int>(size, INT32_MIN);

    max_level_vertex.reserve(size);
    for (int i = 0; i < size; ++i) {
        max_level_vertex.push_back(i);
    }
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

    path_parent[left_child] = path_parent[node];
    path_parent[node] = -1;

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

    path_parent[right_child] = path_parent[node];
    path_parent[node] = -1;

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

int LinkCutTree::Access(int node) {
    // returns last encountered path_parent

    Splay(node);

    if (splay_right[node] != -1) {
        path_parent[splay_right[node]] = node;
        splay_parent[splay_right[node]] = -1;
        splay_right[node] = -1;
        Update(node);
    }

    int last_path_parent = node;
    while (path_parent[node] != -1) {
        int current_path_parent = path_parent[node];
        last_path_parent = current_path_parent;
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

    return last_path_parent;
}

int LinkCutTree::Root(int node) {
    Access(node);
    while(splay_left[node] != -1) {
        node = splay_left[node];
    }
    Splay(node);

    return node;
}

int LinkCutTree::Parent(int node) {
    Splay(node);

    if (splay_left[node] == -1) {
        // node is on top of its preferred path
        return path_parent[node];
    }

    node = splay_left[node];
    while(splay_right[node] != -1) {
        node = splay_right[node];
    }
    Splay(node);

    return node;
}

void LinkCutTree::LinkToRoot(int future_child, int future_parent) {
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
    Update(future_child);
    Update(future_parent);
}

void LinkCutTree::Cut(int node) {
    Access(node);
    splay_parent[splay_left[node]] = -1;
    splay_left[node] = -1;
    Update(node);
}

/*void LinkCutTree::MakeRoot(int node) {
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
}*/

void LinkCutTree::Update(int node) {
    // updates the max_level_edge associated to node

    max_level_vertex[node] = node;

    if (splay_left[node] != -1) {
        if (level[max_level_vertex[splay_left[node]]] > level[max_level_vertex[node]]) {
            max_level_vertex[node] = max_level_vertex[splay_left[node]];
        }
    }
    if (splay_right[node] != -1) {
        if (level[max_level_vertex[splay_right[node]]] > level[max_level_vertex[node]]) {
            max_level_vertex[node] = max_level_vertex[splay_right[node]];
        }
    }
}

int LinkCutTree::MaxLevelVertex(int first, int second) {
    // returns vertex of maximal level on the path between the input vertices

    if (Root(first) != Root(second)) {
        return -1;
    }
    if (first == second) {
        return first;
    }

    Access(first);
    int least_common_ancestor = Access(second);

    if (least_common_ancestor == second) {
        Splay(first);
        if (level[max_level_vertex[first]] > level[least_common_ancestor]) {
            return max_level_vertex[first];
        }
        return least_common_ancestor;
    }
    if (least_common_ancestor == first) {
        Access(first);
        Splay(second);
        if (level[max_level_vertex[second]] > level[least_common_ancestor]) {
            return max_level_vertex[second];
        }
        return least_common_ancestor;
    }

    Splay(first);
    int first_candidate = max_level_vertex[first];

    Access(first);
    Splay(second);
    int second_candidate = max_level_vertex[second];

    if (first_candidate == -1){
        return second_candidate;
    }
    if (second_candidate == -1) {
        return first_candidate;
    }

    if (level[first_candidate] > level[second_candidate]) {
        return first_candidate;
    }
    return second_candidate;
}

void LinkCutTree::UpdateLevel(int node, int new_level) {
    Access(node);
    level[node] = new_level;
    Update(node);
}
