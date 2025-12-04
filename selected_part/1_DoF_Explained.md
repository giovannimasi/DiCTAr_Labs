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
The syntax is `G = c2d(Gcont, Ts, 'zoh')`; where Gcont is our plant in C.T., Ts is the sampling time and zoh is the zero order hold filter 
> [!NOTE]
> You don't really care about zoh method, is just the method used for the discretization. If you are more interested: [Matlab c2d](https://it.mathworks.com/help/ident/ref/dynamicsystem.c2d.html)

> [!TIP]
> Before proceeding, it's a good practice to understand how G is made. In particular, with the function `zpkdata` we can see what are zeros, poles and gain of our plant.
> Syntax `[zG, pG, kG] = zpkdata(G, 'v')`

At this point, before starting with the analysis, let's divide our plant in two polynomials, in particular the numerator of G (B) and the denominator of G (A).
The structure of G will be:  
$G(z) = \frac{B(z)}{A(z)}$

## Analyse steady state requirements
Now real analysis starts.  
In this case, we are interested in computing steady state requirements, that means the requirements that our controller must satisfy when all the transitories are ended.  
In this section we have to concentrate on the number of poles in $z=1$ that our plant G has and our controller C must have.
Let's start by our plant, we can easily see the number of using Matlab function `zpkdata`.  
The syntax is `[zG, pG, kG = zpkdata(G, 'v')`

