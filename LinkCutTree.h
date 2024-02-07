#ifndef K_FORESTS_LINKCUTTREE_H
#define K_FORESTS_LINKCUTTREE_H


#include <vector>

class LinkCutTree {
public:
    explicit LinkCutTree(int size);

    int Root(int node);

    void Cut(int node);

    void Link(int first, int second);

private:
    std::vector<int> splay_parent;
    std::vector<int> path_parent;
    std::vector<int> splay_left;
    std::vector<int> splay_right;

    void Splay(int node);

    void SplayRotateLeft(int node);

    void SplayRotateRight(int node);

    bool IsLeftChild(int node);

    int Access(int node);

    void LinkToRoot(int future_child, int future_parent);

    void MakeRoot(int node);
};


#endif //K_FORESTS_LINKCUTTREE_H
