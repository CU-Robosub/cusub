#!/bin/bash
DIR=$(pwd | grep -o "\w*-*$")
if [ "$DIR" != "Final_Build" ]; then
    cd ..
fi
cd src/
for d in */; do
    cd $d
    git add *
    git commit -m "SUB AUTO COMMIT"
    git push origin master
    cd ..
done
