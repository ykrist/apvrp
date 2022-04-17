    map(to_entries) 
  | add 
  | group_by(.key) 
  | map({key: .[0].key, value: map(.value) | max } ) 
  | sort_by(.key | tonumber)
  | from_entries
