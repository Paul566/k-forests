#ifndef K_FORESTS_TESTER_H
#define K_FORESTS_TESTER_H

#include "GraphicMatroid.h"


class Tester {
public:
    // statistics of the last RunWithoutChecking
    double runtime;
    int optimal_size;
    int num_augmentations;
    int shortest_augmentation_length;
    int longest_augmentation_length;
    int num_find_edge_levels;
    int num_find_edge_bfi;  // number of "find next edge of maximal level" in BlockFlowIndependence;


    Tester(const std::vector<std::vector<int>> &adj_list, const std::vector<int> &ground_truth_sizes,
           std::string initialization_style);

    double RunTest(int k);

    std::vector<double> RunForAllK();

    void RunWithoutChecking(int k);

private:
    std::vector<std::vector<int>> adj_list_;
    std::vector<int> ground_truth_sizes_;
    std::string initialization_style_;

    bool ForestsAreDisjoint(const std::vector<std::vector<std::pair<int, int>>> &forests);

    static int SumOfSizes(const std::vector<std::vector<std::pair<int, int>>> &forests);

    bool IsAcyclic(const std::vector<std::pair<int, int>> &forest);
};


#endif //K_FORESTS_TESTER_H
