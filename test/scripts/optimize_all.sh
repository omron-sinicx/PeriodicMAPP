#!/bin/bash
environments=(1x1 2x1 2x2 2x2large 6small 6)
for ((i=0;i<6;i++)); do
    env=${environments[$i]}
    echo $env
    cd $env
    bash ../scripts/optimize.sh
    cd ..
done
