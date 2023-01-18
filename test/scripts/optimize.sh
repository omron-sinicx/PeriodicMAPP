#!/bin/bash

for ((i=1; i<4 ;i++)); do
    ../../build/generate_path_and_intersection $i < env.txt > condition_$i.txt
    ../../build/generate_general_initial_periodic_plan 1.0 < condition_$i.txt > initial_$i.txt
    cp initial_$i.txt initial.txt
    { time ../../build/optimize_periodic_plan_g2o ../config/optimization_config.yaml; }  2> log_opt_$i.txt
    rm initial.txt
    cp optimized.txt optimized_$i.txt
done

