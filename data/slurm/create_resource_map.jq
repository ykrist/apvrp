sort_by(.ArrayTaskID) 
  | map(select(.output) 
  | { (.ArrayTaskID | tostring): .output }) 
  | add
