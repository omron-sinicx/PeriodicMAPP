#!/bin/bash
environments=(1x1 2x1 2x2 2x2large 6small 6)

for ((i=0;i<6;i++)); do
    env=${environments[$i]}
    echo $env
    cd $env
    bash ../scripts/test.sh > log_test.txt
    bash ../scripts/assign.sh 
    python3 ../scripts/evaluate.py > evaluation.txt
    bash ../scripts/test_2.sh > log_test_2.txt
    bash ../scripts/assign_2.sh 
    python3 ../scripts/evaluate_2.py > evaluation_2.txt
    cd ..
done
