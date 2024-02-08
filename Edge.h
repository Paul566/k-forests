#ifndef K_FORESTS_EDGE_H
#define K_FORESTS_EDGE_H


class Edge {
public:
    int forest;
    // the index of a forest this edge is in, -1 if not covered
    // forest are enumerated from 0
    // TODO maybe use std::optional<int> instead of the magical constant "-1"
    int level;

    Edge(int from, int to) : from_(from), to_(to) {
        forest = -1;
        level = INT32_MAX;
    }

    int AnotherVertex(int vertex) const {
        if (vertex == to_) {
            return from_;
        }
        return to_;
    }

    std::pair<int, int> Vertices() {
        return {from_, to_};
    }

private:
    const int from_, to_;
};


#endif //K_FORESTS_EDGE_H
