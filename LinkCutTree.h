#ifndef K_FORESTS_LINKCUTTREE_H
#define K_FORESTS_LINKCUTTREE_H


#include <vector>
#include <memory>
#include <unordered_map>

struct PairHash {
    // TODO is this needed?
    template<class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &pair) const {
        auto hash_first = std::hash<T1>{}(pair.first);
        auto hash_second = std::hash<T2>{}(pair.second);

        if (hash_first != hash_second) {
            return hash_first ^ hash_second;
        }

        return hash_first;
    }
};


class LinkCutTree {
public:
    explicit LinkCutTree(int size);

    int Root(int node);

    int Parent(int node);

    void Cut(int node);

    void LinkToRoot(int future_child, int future_parent);

    void Link(int first, int second);

    int MaxLevelVertex(int first, int second);

    void UpdateLevel(int node, int new_level);

private:
    std::vector<int> splay_parent;
    std::vector<int> path_parent;
    std::vector<int> splay_left;
    std::vector<int> splay_right;
    std::vector<int> max_level_vertex;
    // max_level_edges[i] is the vertex of maximal level in the node's subtree in its splay tree
    std::vector<int> level;
    // level[i] is the level of the i-th vertex
    std::vector<bool> reversed;

    void Splay(int node);

    void SplayRotateLeft(int node);

    void SplayRotateRight(int node);

    bool IsLeftChild(int node);

    int Access(int node);

    void MakeRoot(int node);

    void Update(int node);

    void Toggle(int node);

    void Push(int node);
};


#endif //K_FORESTS_LINKCUTTREE_H
