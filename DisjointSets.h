#ifndef K_FORESTS_DISJOINTSETS_H
#define K_FORESTS_DISJOINTSETS_H


#include <vector>

class DisjointSets {
public:
    explicit DisjointSets(int size);

    int Representative(int element);

    void Unite(int first, int second);

private:
    std::vector<int> ranks;
    std::vector<int> parents;

    void UniteRepresentatives(int first, int second);
};


#endif //K_FORESTS_DISJOINTSETS_H
