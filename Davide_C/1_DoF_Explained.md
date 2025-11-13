# 1 DoF controller - exercise explaination
### Flowchart
1. Compute discrete time G
2. Analyse steady state requirements
3. Analyse transient response requirements
4. Choose what zero-pole we want to cancel
- define Aplus, Aminus, Bplus, Bminus
- compute degree of R, S and Am
6. Define controller poles and Am
7. Define, if present, extra equation
8. Build Adioph, Bdioph and Ms
9. Solve diophantine equation
10. Build the controller
11. Simulate and check results

## Compute discrete time G
In our exercise, G is given in continuous time, so, in order to realise our controller C, we must discretise it. We can easily do it thanks to Matlab function `c2d`.  
The syntax is `C = c2d(Gcont, Ts, 'zoh')`; where Gcont is our plant in C.T., Ts is the sampling time and zoh is the zero order hold filter 
> Note: you don't really care about zoh, is just the method used for the discretization. If you are more interested: [Matlab c2d](https://it.mathworks.com/help/ident/ref/dynamicsystem.c2d.html)
