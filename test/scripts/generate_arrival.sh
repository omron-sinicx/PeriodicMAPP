#!/bin/bash
environments=(1x1 2x1 2x2 2x2large 6small 6)
densities=(0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0 2.25 2.5)
iteration=10
time=1000
time2=100
Ns=(2 3 4 4 6 6)
for ((i=0;i<6;i++)); do
    env=${environments[$i]}
    N=${Ns[$i]}
    echo $env
    echo $N
    cd $env
    mkdir -p inputs
    mkdir -p inputs2
    for ((j=0;j<10;j++)); do
	density=${densities[$j]}
	echo $density
	for ((ite=0; ite<$iteration; ite++)); do
	    let seed=100*$i+10*$j+$ite
	    echo $seed
	    arrival=inputs/arrival-$density-$ite.txt
	    arrival2=inputs2/arrival-$density-$ite.txt
            python3 ../../tools/generate_exponential_distributions.py $N $density $time $seed > $arrival
            python3 ../../tools/generate_exponential_distributions.py $N $density $time2 $seed > $arrival2
	done
    done    
    cd ..
done
