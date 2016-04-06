#!/bin/bash

RUN=ann_quad.py

for i in $(eval echo {$1..$2}) 
do
    ./run_launcher.sh $i 2000 50 $RUN 
done
