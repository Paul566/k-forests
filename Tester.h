#ifndef K_FORESTS_TESTER_H
#define K_FORESTS_TESTER_H

#include "GraphicMatroid.h"


class Tester {
public:
    Tester(const std::vector<std::vector<int>>& adj_list, const std::vector<int>& ground_truth_sizes);

    double RunTest(int k);

    std::vector<double> RunForAllK();

private:
    std::vector<std::vector<int>> adj_list_;
    std::vector<int> ground_truth_sizes_;

    bool ForestsAreDisjoint(const std::vector<std::vector<std::pair<int, int>>>& forests);

    int SumOfSizes(const std::vector<std::vector<std::pair<int, int>>>& forests);

    bool IsAcyclic(const std::vector<std::pair<int, int>>& forest);
};


#endif //K_FORESTS_TESTER_H
