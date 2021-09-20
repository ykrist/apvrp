#!/bin/bash

mkdir -p run run-dbg
rm -f run/* run-dbg/*

for F in $(ls templates/*) ; do
    for S in $(cat seeds.txt) ; do
        NAME="$(jq -r .param_name $F)_$S"
        echo $NAME
        DEST=run/${NAME}.json
        jq ".gurobi += [[\"Seed\", {\"Int\": $S}]] | .param_name=\"$NAME\"" $F > run/${NAME}.json
        jq '.timelimit=180000' run/${NAME}.json > run-dbg/${NAME}.json
    done
done
