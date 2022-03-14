#!/bin/sh

g++ --std=c++11 -fopenmp main.cpp -o parallel.out
g++ --std=c++11 main.cpp -o sequential.out

echo "Sequential"
./sequential.out < $1

echo "------------------------------------------------------------------\n"

echo "Parallel"
./parallel.out < $1
