#!/bin/bash
densities="0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0 2.25 2.5"
iteration=10

mkdir -p results2

for density in $densities; do
    echo $density
    for ((ite=0; ite<$iteration; ite++)); do
	large=inputs2/large-$density-$ite.txt
	cp $large in.txt
	../../../mapf-bench/build/planner_benchmark ../config/PSIPP_config.yaml
	cp out.txt results2/PSIPP-$density-$ite.txt
	small=inputs2/small-$density-$ite.txt
	cp $small in.txt
	../../../mapf-bench/build/planner_benchmark ../config/CCBS_config.yaml
	cp out.txt results2/CCBS-$density-$ite.txt
    done
done

