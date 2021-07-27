#!/bin/bash

mkdir -p run
rm -f run/*

for F in $(ls templates/*) ; do
    for S in $(cat seeds.txt) ; do
        NAME="$(jq -r .param_name $F)_$S"
        echo $NAME
        jq ".gurobi += [[\"Seed\", {\"Int\": $S}]] | .param_name=\"$NAME\"" $F > run/${NAME}.json
    done
done