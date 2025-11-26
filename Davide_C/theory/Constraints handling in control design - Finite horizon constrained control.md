# Constraints handling in control design
Physical limitations of the actuator device impose _hard constraints_ on the control input $u(t)$. For example, a possible constraint can be a **saturation constraint** of the form   
$$-u_M \leq u(t) \leq u_{M}, \, \forall t \geq 0$$  
$$\Rightarrow |u(t)| \leq u_{M}, \, \forall t \geq 0$$  
The saturation constraint can be described as a _nonlinear static function_ of the control input as  

m(t) = u(t)    if -u_M ≤ u(t) ≤ u_M  
m(t) = -u_M    if u(t) < -u_M  
m(t) = u_M     if u(t) > u_M  

m(t) → saturated input

Note that, when -u_M ≤ u(t) ≤ u_M, m(t) = u(t)

When the input saturation is active (i.e. when either $u(t) > u_M$ or $u(t) < - u_M$), the feedback control system is non-linear.  
In the presence of saturation, control system work without feedback  
Exceeding the input prescribed bounds leads to unexpected sysmet behaviours such as large overshoots, low performace, or, in the worst case, instability🙀

### How we can handle this situation?
Input saturation constraint cannot be handled directly by neither pole placement nor optimal control design procedures, but it must be checked _a posteriori_ through simulation.  
If such a requirement is not met, a common procedure is to **slow down the transient response of the control system**, e.g., by choosing **dominant closed loop poles with larger time constants**.  
However, an increase of the closed loop dominant time constant may cause a degradation of the transient performance (i.e., rise time and settling time) &rarr; conflicting requirements.

### Approches to constrained control
- **Caution** &rarr; back off performance demands so constraints are met &rarr; slow down the controller
- **Serendipitous** &rarr; allow occasional constraint violation (i.e. saturate the control input without modifying the original design) &rarr; used if the difference between input and saturation is small
- **Evolutionary** &rarr; begin with a linear design and add embellishments, for example, anti-windup &rarr; build an additional structure to handle saturation
- **Tactical** &rarr; include constraints from the beginning, for example, Model Predictive Control

## Finite horizon constrained control
We need both discrete time approach and optimal control. Because in discrete time we can handle the situation in **discrete time instant** and not in continuous time.  
The starting point is considering discrete time, finite horizon, linear quadratic optimal control.  
Let's break it down step by step:  
- first of all, we consider the solution in the absence of input constraints  
- The discrete LTI system dynamics are represented by $x(k+1) = A x(k) + B u(k)$.  
  min_U(k) J(x(k), U(k)) = min_U(k) [∑_(i=0)^(H_p-1) xᵀ(k+i)Qx(k+i) + uᵀ(k+i)Ru(k+i) 
                         + xᵀ(k+H_p)Sx(k+H_p)]

  U(k) = [u(k) u(k+1) ... u(k+H_p-1)]

  $H_p \to$ time prediction horizon
- In discrete time, the unconstrained optimal solution $U^*(k)$ can be easily computed through a finite dimensional quadratic optimization problem
- my objective is to compute the sequence $U(k) = [u(k) \\ \\ u(k+1) \\ \\ u(k+2)]$ &rarr; we express the function as a function of $U(k)$ and the initial state $x(k)$
- (computation... &rarr; see slides for math passages)
- so, we obtained a cost function $J$ that depends only on $x(k)$ and $U(k)$
- 
