#ifndef K_FORESTS_SEGMENTTREE_H
#define K_FORESTS_SEGMENTTREE_H


#include <vector>
#include <unordered_set>

class SegmentTree {
public:
    SegmentTree(std::vector<int>& left_ends, std::vector<int>& right_ends);

    std::vector<int> Retrieve(int first, int second);

private:
    int min_left;
    int max_right;

    const std::vector<int> segment_left_end;
    const std::vector<int> segment_right_end;

    std::vector<int> left_child;
    std::vector<int> right_child;
    std::vector<int> vertex_left_end;
    std::vector<int> vertex_right_end;
    // 0 is root
    std::vector<std::unordered_set<int>> segment_indices;
    // segment_indices[i] contains indices of segments such that
    // segment_left_end <= vertex_left_end[i] <= vertex_right_end[i] <= segment_right_end,
    // but it's not true for the parent of i

    int Build(int left, int right);

    void Insert(int segment_index, int vertex);

    void Delete(int segment_index, int vertex);

    void SegmentsContainingPoint(int point, int vertex, std::unordered_set<int>& output);
};


#endif //K_FORESTS_SEGMENTTREE_H
