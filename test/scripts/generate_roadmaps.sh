#!/bin/bash
environments=(1x1 2x1 2x2 2x2large 6small 6)
densities=(0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0 2.25 2.5)
iteration=10

for ((i=0;i<6;i++)); do
    env=${environments[$i]}
    echo $env
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
	    sed -i "5i \ \ seed: $seed" ../config/large_roadmap_config.yaml 
	    ../../../mapf-bench/build/roadmap_generation ../config/large_roadmap_config.yaml
	    sed -i "5d" ../config/large_roadmap_config.yaml 
	    python3 ../../tools/add_corridor.py large_roadmap.txt 1000 > large_roadmap_with_corridor.txt
	    large=inputs/large-$density-$ite.txt
	    python3 ../../tools/generate_online_instance.py large_roadmap_with_corridor.txt $arrival > $large
	    large2=inputs2/large-$density-$ite.txt
	    python3 ../../tools/generate_online_instance.py large_roadmap_with_corridor.txt $arrival2 > $large2
	    sed -i "5i \ \ seed: $seed" ../config/small_roadmap_config.yaml 
	    ../../../mapf-bench/build/roadmap_generation ../config/small_roadmap_config.yaml
	    sed -i "5d" ../config/small_roadmap_config.yaml 
	    python3 ../../tools/add_corridor.py small_roadmap.txt 5 > small_roadmap_with_corridor.txt
	    small=inputs2/small-$density-$ite.txt
	    python3 ../../tools/generate_online_instance.py small_roadmap_with_corridor.txt $arrival2 > $small
	done
    done    
    cd ..
done
