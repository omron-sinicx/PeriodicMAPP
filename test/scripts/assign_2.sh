#!/bin/bash
densities="0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0 2.25 2.5"
iteration=10

for density in $densities; do
    echo $density
    for ((ite=0; ite<$iteration; ite++)); do
	arrival=inputs2/arrival-$density-$ite.txt
	for ((i=1; i<4 ;i++)); do
	    python3 ../../tools/assignment.py optimized_$i.txt $arrival 5 > results2/Periodic-$i-$density-$ite.txt
	done
    done
done
