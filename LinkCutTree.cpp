#include <stdexcept>
#include <queue>
#include "LinkCutTree.h"

LinkCutTree::LinkCutTree(int size) {
    splay_parent = std::vector<int>(size, -1);
    path_parent = std::vector<int>(size, -1);
    splay_left = std::vector<int>(size, -1);
    splay_right = std::vector<int>(size, -1);
}

void LinkCutTree::SplayRotateLeft(int node) {
    // brings left child of node to the top

    int left_child = splay_left[node];
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
}

void LinkCutTree::SplayRotateRight(int node) {
    // brings right child of node to the top

    int right_child = splay_right[node];
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

void LinkCutTree::LinkToRoot(int future_child, int future_parent) {
    // future_child has to be the root of its represented tree

    if (Root(future_child) != future_child) {
        throw std::runtime_error("in Link: future_child has to be the root of its tree");
    }

    if (future_child == Root(future_parent)) {
        return;
    }

    Access(future_child);
    Access(future_parent);
    splay_left[future_child] = future_parent;
    splay_parent[future_parent] = future_child;
}

void LinkCutTree::Cut(int node) {
    Access(node);
    splay_parent[splay_left[node]] = -1;
    splay_left[node] = -1;
}

void LinkCutTree::Link(int first, int second) {
    // links nodes first and second, second becomes parent

    if (Root(first) == Root(second)) {
        return;
    }

    MakeRoot(first);
    LinkToRoot(first, second);
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

}
