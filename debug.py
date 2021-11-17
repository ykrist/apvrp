from gurobi import *

m = read("broken-base.lp")
vars, cons = m.getVars(), m.getConstrs()
vars = { v.VarName : v for v in vars}
cons = { c.ConstrName : c for c in cons }

m.optimize()
m.setParam("IISMethod", 1)
m.computeIIS()

iis_cons = { k : v for k,v in cons.items() if v.IISConstr > 0 }
iis_bounds = { k : v for k,v in vars.items() if v.IISUB > 0 or v.IISLB > 0 }