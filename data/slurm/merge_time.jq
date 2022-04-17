map(select(length > 0))
  | length as $N
  | map(to_entries) 
  | add 
  | group_by(.key) 
  | map(select(length == $N or $N == 1) | {key: .[0].key, value: map(.value) | max } ) 
  | sort_by(.key | tonumber)
  | from_entries
