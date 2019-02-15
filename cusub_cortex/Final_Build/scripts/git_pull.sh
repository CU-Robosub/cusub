#!/bin/bash
DIR=$(pwd | grep -o "\w*-*$")
if [ "$DIR" != "Final_Build" ]; then
    cd ..
fi
cd src/
for d in */; do
    cd $d
    git pull origin master
    cd ..
done
