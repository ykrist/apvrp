#!/bin/bash

mkdir -p run
rm -f run/*

for F in $(ls templates/*) ; do
    I=0
    for S in $(cat seeds.txt) ; do
        NAME="$(jq -r .param_name $F)_$I"
        I=$(( I + 1 ))
        echo $NAME
        DEST=run/${NAME}.json
        jq ".gurobi += [[\"Seed\", {\"Int\": $S}]] | .param_name=\"$NAME\"" $F > run/${NAME}.json
    done
done
