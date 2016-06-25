KDTree
Makefile is present to build all the applications. 

To create build_kdtree:
make build_kdtree
or
g++ -Wall -std=c++11    build_kdtree.cpp -o build_kdtree

To create query_kdtree:
make query_kdtree
or
g++ -Wall -std=c++11    query_kdtree.cpp -o query_kdtree

To run build_kdtree:
./build_kdtree <sample_data.csv> <output_tree.csv>

To run query_kdtree:
./query_kdtree <query_data.csv> <tree_from_previous_step.csv> <output_result.csv> 

// Main library files. 
Helper.h
Node.h
KDTree.h

// Application builder files
build_kdtree.cpp
query_kdtree.cpp

// Testing code
main.cpp
Tester.h // runs and compares result against brute force method. 
testTree.py // Generates sample datapoints

kdtree.pdf contain the report for this exercise.
