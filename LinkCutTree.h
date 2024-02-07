#ifndef K_FORESTS_LINKCUTTREE_H
#define K_FORESTS_LINKCUTTREE_H


#include <vector>
#include <memory>
#include "Edge.h"


class LinkCutTree {
public:
    explicit LinkCutTree(int size);

    int Root(int node);

    void Cut(int node);

    void Link(int first, int second, const std::shared_ptr<Edge>& edge);

    std::shared_ptr<Edge> MaxLevelEdge(int first, int second);

private:
    std::vector<int> splay_parent;
    std::vector<int> path_parent;
    std::vector<int> splay_left;
    std::vector<int> splay_right;
    std::vector<std::shared_ptr<Edge>> edges;
    // edges[i] is the edge that goes up from i-th vertex in the represented tree
    std::vector<std::shared_ptr<Edge>> max_level_edges;
    // max_level_edges[i] is the edge of maximal level associated to some vertex
    // in the node's subtree in its splay tree

    void Splay(int node);

    void SplayRotateLeft(int node);

    void SplayRotateRight(int node);

    bool IsLeftChild(int node);

    void Access(int node);

    void LinkToRoot(int future_child, int future_parent, const std::shared_ptr<Edge>& edge);

    void MakeRoot(int node);

    void ReversePath(int node);

    void UpdateSplaySubTree(int node);

    void Update(int node);
};


#endif //K_FORESTS_LINKCUTTREE_H
