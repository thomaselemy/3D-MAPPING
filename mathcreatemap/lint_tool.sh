#!/bin/bash

FILE="lint.txt"

echo Output File: $FILE

BIG_OPTIONS="-format-style='none' main.cpp georef.cpp math.hpp georef.hpp -- -std=c++14"

echo Using following options | tee $FILE
echo

clang-tidy-6.0 -dump-config | tee -a $FILE

echo
echo Starting General Clean | tee -a $FILE
echo

clang-tidy-6.0 $BIG_OPTIONS  | tee -a $FILE

echo Done | tee -a $FILE
sleep 5
