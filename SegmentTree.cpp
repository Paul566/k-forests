#include <cstdint>
#include <stdexcept>
#include <iostream>
#include "SegmentTree.h"

SegmentTree::SegmentTree(std::vector<int> &left_ends, std::vector<int> &right_ends) :
        segment_left_end(left_ends), segment_right_end(right_ends) {
    if (left_ends.empty()) {
        Build(0, 1);
        return;
    }

    min_left = INT32_MAX;
    max_right = INT32_MIN;
    for (int i = 0; i < static_cast<int>(left_ends.size()); ++i) {
        if (left_ends[i] > right_ends[i]) {
            std::swap(left_ends[i], right_ends[i]);
        }
        if (left_ends[i] < min_left) {
            min_left = left_ends[i];
        }
        if (right_ends[i] > max_right) {
            max_right = right_ends[i];
        }
    }

    Build(min_left, max_right);

    for (int i = 0; i < static_cast<int>(left_ends.size()); ++i) {
        Insert(i, 0);
    }
}

int SegmentTree::Build(int left, int right) {
    int this_index = static_cast<int>(vertex_left_end.size());
    vertex_left_end.push_back(left);
    vertex_right_end.push_back(right);
    left_child.push_back(-1);
    right_child.push_back(-1);
    segment_indices.emplace_back();

    if (left + 1 < right) {
        int middle = (left + right) / 2;
        left_child[this_index] = Build(left, middle);
        right_child[this_index] = Build(middle, right);
    }

    return this_index;
}

void SegmentTree::Insert(int segment_index, int vertex) {
    if ((segment_left_end[segment_index] <= vertex_left_end[vertex]) &&
        (vertex_right_end[vertex] <= segment_right_end[segment_index])) {
        segment_indices[vertex].insert(segment_index);
    } else {
        int middle = (vertex_left_end[vertex] + vertex_right_end[vertex]) / 2;
        if (segment_left_end[segment_index] < middle) {
            Insert(segment_index, left_child[vertex]);
        }
        if (segment_right_end[segment_index] > middle) {
            Insert(segment_index, right_child[vertex]);
        }
    }
}

void SegmentTree::Delete(int segment_index, int vertex) {
    if ((segment_left_end[segment_index] <= vertex_left_end[vertex]) &&
        (vertex_right_end[vertex] <= segment_right_end[segment_index])) {
        segment_indices[vertex].erase(segment_index);
    } else {
        int middle = (vertex_left_end[vertex] + vertex_right_end[vertex]) / 2;
        if (segment_left_end[segment_index] < middle) {
            Delete(segment_index, left_child[vertex]);
        }
        if (segment_right_end[segment_index] > middle) {
            Delete(segment_index, right_child[vertex]);
        }
    }
}

std::vector<int> SegmentTree::Retrieve(int first, int second) {
    // returns indices of the segments that have exactly one endpoint in {first + 1, ..., second}
    // then deletes those segments
    if (second < first) {
        std::swap(first, second);
    }

    int vertex = 0;
    while (((vertex_right_end[left_child[vertex]] > second) && (left_child[vertex] != -1)) ||
           ((vertex_left_end[right_child[vertex]] <= first) && (right_child[vertex] != -1))) {
        if ((vertex_right_end[left_child[vertex]] > second) && (left_child[vertex] != -1)) {
            vertex = left_child[vertex];
            continue;
        }
        if ((vertex_left_end[right_child[vertex]] <= first) && (right_child[vertex] != -1)) {
            vertex = right_child[vertex];
            continue;
        }
        break;
    }
    // now vertex is the lowest vertex that contains both break lines

    std::unordered_set<int> contain_first;
    SegmentsContainingPoint(first, vertex, contain_first);
    std::unordered_set<int> contain_second;
    SegmentsContainingPoint(second, vertex, contain_second);

    std::vector<int> answer;
    for (int index: contain_first) {
        if (contain_second.find(index) == contain_second.end()) {
            answer.push_back(index);
            Delete(index, vertex);
        }
    }
    for (int index: contain_second) {
        if (contain_first.find(index) == contain_first.end()) {
            answer.push_back(index);
            Delete(index, vertex);
        }
    }

    return answer;
}

void SegmentTree::SegmentsContainingPoint(int point, int vertex, std::unordered_set<int> &output) {
    // returns segments that have left_end <= point and right_end > point
    if (vertex == -1) {
        return;
    }
    if ((vertex_left_end[vertex] > point) || (vertex_right_end[vertex] <= point)) {
        return;
    }

    output.insert(segment_indices[vertex].begin(), segment_indices[vertex].end());
    SegmentsContainingPoint(point, left_child[vertex], output);
    SegmentsContainingPoint(point, right_child[vertex], output);
}
