# Notes on cycle cuts

## PV cycles 
Bog-standard cycle cut, no finding chains or anything like that, just summing over PVs.
Enabled if `cycle_cuts` is true.

## AV Cycles

Enabled if `cycle_cuts` is true

If an illegal chain within the cycle is found, add fork cuts if the chain is less than 3 tasks (2 Yvars) else add a tournament cut.