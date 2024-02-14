#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "GraphicMatroid.h"
#include "Tester.h"
#include "DisjointSets.h"
#include "LinkCutTree.h"


std::vector<std::vector<int>> ReadAdjList(const std::string &path) {
    std::vector<std::vector<int>> adj_list;
    std::fstream input_file(path);

    if (input_file.is_open()) {
        std::string line;
        std::getline(input_file, line);

        int num_vertices, num_edges;
        std::istringstream stream(line);
        stream >> num_vertices >> num_edges;

        for (int i = 0; i < num_vertices; ++i) {
            std::getline(input_file, line);
            std::vector<int> neighbors;

            stream = std::istringstream(line);
            int next_vertex;
            while (stream >> next_vertex) {
                neighbors.push_back(next_vertex - 1);
            }

            adj_list.push_back(neighbors);
        }
    } else {
        throw std::runtime_error("input file " + path + " not found");
    }
    input_file.close();

    return adj_list;
}

std::vector<int> ReadAnswers(const std::string &path) {
    std::vector<int> answers;
    std::fstream input_file(path);

    if (input_file.is_open()) {
        int next_answer;
        while (input_file >> next_answer) {
            answers.push_back(next_answer);
        }
    } else {
        throw std::runtime_error("input file " + path + " not found");
    }

    input_file.close();

    return answers;
}

std::vector<int> SolveForAllK(const std::vector<std::vector<int>> &adj_list, const std::string &initialization_type) {
    // returns (a_0, a_1, ..., a_n), where a_k is the size of the optimal solution for k = k
    std::vector<int> answers;

    for (int k = 0; k <= static_cast<int>(adj_list.size()); ++k) {
        GraphicMatroid graph(adj_list, k, initialization_type);
        graph.GenerateKForests();
        auto forests = graph.GetForests();
        int optimal_size = 0;
        for (const auto &forest: forests) {
            optimal_size += static_cast<int>(forest.size());
        }
        answers.push_back(optimal_size);
    }

    return answers;
}

template<typename T>
void ExportVector(const std::string &path, const std::vector<T> &vector) {
    std::ofstream output(path);
    std::cout << std::fixed;
    std::cout << std::setprecision(4);

    for (T element: vector) {
        output << std::setprecision(4) << element << "\n";
    }

    output.close();
}

void RunRandomTests() {
    std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    std::cout << std::fixed;
    std::cout << std::setprecision(4);

    for (int n = 3; n <= 15; ++n) {
        for (int m = n; m <= n * (n - 1) / 2; ++m) {
            std::string filename_graph = std::to_string(n) + "-" + std::to_string(m) + ".txt";
            std::string filename_answers = std::to_string(n) + "-" + std::to_string(m) + "-answers.txt";

            std::cout << filename_graph << ":" << std::endl;

            auto adj_list = ReadAdjList(prefix + filename_graph);
            auto answers = ReadAnswers(prefix + filename_answers);

            Tester tester(adj_list, answers, "DFS");
            auto times = tester.RunForAllK();

            for (double time: times) {
                std::cout << time << " ";
            }
            std::cout << "\n";
        }
    }

    std::cout << "\nAll tests passed!\n";
}

void
SaveStatistics(const std::string &prefix, const std::string &filename_graph, const std::string &init_type, int max_k) {
    auto adj_list = ReadAdjList(prefix + filename_graph);
    Tester tester(adj_list, std::vector<int>(), init_type);

    std::vector<double> times;
    std::vector<int> sizes;
    std::vector<int> shortest_aug_lengths;
    std::vector<int> longest_aug_lengths;
    std::vector<int> num_augmentations;
    std::vector<int> num_find_edge_levels;
    std::vector<int> num_find_edge_bfi;
    std::cout << "graph: " << filename_graph << "\ninit_type: " << init_type << std::endl;
    std::cout << "k\ttime(s)\toptimal_size\tshortest_aug_length\tlongest_aug_length\tnum_augs\t"
              << "FindEdges_Levels\tFindEdges_BFI" << std::endl;
    for (int k = 1; k <= max_k; ++k) {
        tester.RunWithoutChecking(k);
        std::cout << std::setprecision(4) << k << "\t" << tester.runtime << "\t" << tester.optimal_size << "\t"
                  << tester.shortest_augmentation_length << "\t" << tester.longest_augmentation_length << "\t"
                  << tester.num_augmentations << "\t" << tester.num_find_edge_levels << "\t" << tester.num_find_edge_bfi
                  << "\t" << std::endl;
        times.push_back(tester.runtime);
        sizes.push_back(tester.optimal_size);
        shortest_aug_lengths.push_back(tester.shortest_augmentation_length);
        longest_aug_lengths.push_back(tester.longest_augmentation_length);
        num_augmentations.push_back(tester.num_augmentations);
        num_find_edge_levels.push_back(tester.num_find_edge_levels);
        num_find_edge_bfi.push_back(tester.num_find_edge_bfi);
    }
    std::cout << "-------------------------------------------------------------------------------" << std::endl;

    ExportVector(prefix + "times-" + init_type + ".txt", times);
    ExportVector(prefix + "sizes.txt", sizes);
    ExportVector(prefix + "shortest-aug-length-" + init_type + ".txt", shortest_aug_lengths);
    ExportVector(prefix + "longest-aug-length-" + init_type + ".txt", longest_aug_lengths);
    ExportVector(prefix + "num-augmentations-" + init_type + ".txt", num_augmentations);
    ExportVector(prefix + "num-find-edge-levels-" + init_type + ".txt", num_find_edge_levels);
    ExportVector(prefix + "num-find-edge-bfi-" + init_type + ".txt", num_find_edge_bfi);
}

int main() {

    /*std::string prefix = std::filesystem::current_path().string() + "/../tests/astrophysics/";
    std::string filename_graph = "astrophysics.txt";
    int max_k = 30;
    SaveStatistics(prefix, filename_graph, "DFS", max_k);
    SaveStatistics(prefix, filename_graph, "BFS", max_k);
    SaveStatistics(prefix, filename_graph, "random", max_k);*/


    RunRandomTests();

    /*std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    std::string filename_graph = "7-18.txt";
    std::string filename_answers = "7-18-answers.txt";

    auto adj_list = ReadAdjList(prefix + filename_graph);
    auto answers = ReadAnswers(prefix + filename_answers);

    GraphicMatroid graph(adj_list, 2, "DFS");
    graph.GenerateKForests();
    graph.PrintGraph();*/

    /*LinkCutTree lct(7);
    lct.Link(0, 1);
    lct.Link(5, 6);
    lct.Link(0, 6);
    lct.Link(0, 4);
    lct.Link(2, 3);
    lct.Link(1, 2);
    lct.Cut(0);

    for (int i = 0; i <= 6; ++i) {
        lct.UpdateLevel(i, i);
    }

    std::cout << lct.Parent(0) << std::endl << std::endl;

    for (int i = 0; i <= 6; ++i) {
        for (int j = i; j <= 6; ++j) {
            std::cout << "(" << i << ", " << j << "): " << lct.MaxLevelVertex(i, j) << std::endl;
        }
    }*/



    return 0;
}
