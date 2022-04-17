#!/bin/bash
set -e 

USAGE="$0 (time|memory) SACCT_JSON RESOURCE_JSON"

function usage {
  echo $USAGE >&2
  exit 1
}

function check_exists {
  if [[ -z $1  ]] ; then 
    usage
  elif [[ ! -f $1 ]] ; then 
    echo "${1} does not exist or is not a file" >&2
    exit 1
  fi
}

case $1 in
  time)
    
  ;;
  memory)

  ;;
  *) usage
  ;; 
esac

SACCT_JSON=$2
RESOURCE_JSON=$3

check_exists "$SACCT_JSON"
check_exists "$RESOURCE_JSON"


LEVELS=( `jq -r ".${1} | map(tostring) | join(\" \")"  levels.json` )

if [[ $1 == "time" ]] ; then 
  sacct-assign-resources -f $SACCT_JSON time --mmin 60 -i ${LEVELS[@]} \
    | jq -s -f create_resource_map.jq > tmp.json
  jq -s -f merge_max.jq $RESOURCE_JSON tmp.json > new.json
  mv new.json $RESOURCE_JSON
  rm tmp.json
else 
  sacct-assign-resources -f $SACCT_JSON memory -i ${LEVELS[@]} \
    | jq -s -f create_resource_map.jq > tmp.json
  jq -s -f merge_max.jq $RESOURCE_JSON tmp.json > new.json
  mv new.json $RESOURCE_JSON
  rm tmp.json
fi
