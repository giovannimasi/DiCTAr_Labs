# Introduction - Discrete-Time Dynamical Systems

## 1. Introduction to Control Systems

### The Role of Automatic Control
Automatic control aims to make a physical quantity (speed, altitude, temperature) behave in a desirable way over time without human intervention.

**Key Definition:**
A **Control System** is a dynamical system behaving in a prescribed way in the absence of human action.

### Basic Definitions
* **Plant:** The dynamical system to be controlled (e.g., a car, a robotic arm).
* **Controlled Output $y(t)$:** The physical variable we want to control (e.g., car speed).
* **Control Input $u(t)$:** The variable that affects the plant behavior and can be manipulated (e.g., throttle valve angle).
* **Disturbance $d(t)$:** External signals affecting the plant dynamics, usually unknown or uncontrollable (e.g., road slope, wind).
* **Reference $r(t)$:** The desired value for the output.

### The Feedback Loop
The core concept is **Feedback**. By comparing the measured output $y(t)$ with the reference $r(t)$, the system generates an error signal $e(t) = r(t) - y(t)$. The controller uses this error to calculate the input $u(t)$ needed to correct the system.

---

## 2. Digital Control Systems

In modern engineering, control laws are rarely implemented via analog circuits (OpAmps). Instead, we use digital computers (microcontrollers, DSPs).

### Structure of a Digital Control Loop
A digital control system interfaces a continuous-time plant with a discrete-time computer.

1.  **Sampler (A/D Converter):** Converts the continuous output $y(t)$ into a sequence of numbers $y(k)$ at discrete time instants $t\_k = k \cdot T\_s$, where $T\_s$ is the **sampling time**.
2.  **Digital Controller:** A computer algorithm that processes the sequence $e(k)$ to generate a control sequence $u(k)$.
3.  **Hold (D/A Converter):** Converts the digital sequence $u(k)$ back into a continuous signal $u(t)$ to drive the actuator. The most common type is the **Zero Order Hold (ZOH)**, which keeps the signal constant between samples.

---

## 3. Discrete-Time Dynamical Systems

Since the computer operates on sequences, we model the system using **Difference Equations** (Discrete time) rather than Differential Equations (Continuous time).

### State-Space Representation
A Linear Time-Invariant (LTI) discrete system is described by:

$$
\begin{cases}
x(k+1) = A\_d x(k) + B\_d u(k) \\
y(k) = C\_d x(k) + D\_d u(k)
\end{cases}
$$

* $x(k) \in \mathbb{R}^n$: State vector at step $k$.
* $u(k) \in \mathbb{R}^m$: Input vector.
* $y(k) \in \mathbb{R}^p$: Output vector.
* $A\_d, B\_d, C\_d, D\_d$: System matrices (discrete).

### Mathematical Proof: Solution of the Discrete-Time System
**Objective:** Find the explicit formula for the state $x(k)$ given an initial condition $x(0)$ and an input sequence $u(0), u(1), \dots$.

**Derivation (Recursive Approach):**

1.  **For $k=0$ calculate $x(1)$:**  

    $$x(1) = A\_d x(0) + B\_d u(0)$$

2.  **For $k=1$ calculate $x(2)$:**
    Substitute the result from step 1 into the state equation:  

    $$x(2) = A\_d x(1) + B\_d u(1)$$
    $$x(2) = A\_d [A\_d x(0) + B\_d u(0)] + B\_d u(1)$$
    $$x(2) = A\_d^2 x(0) + A\_d B\_d u(0) + B\_d u(1)$$

3.  **For $k=2$ calculate $x(3)$:** Substitute the result from step 2 into the state equation:  
   
    $$x(3) = A\_d x(2) + B\_d u(2)$$
    $$x(3) = A\_d [A\_d^2 x(0) + A\_d B\_d u(0) + B\_d u(1)] + B\_d u(2)$$
    $$x(3) = A\_d^3 x(0) + A\_d^2 B\_d u(0) + A\_d B\_d u(1) + B\_d u(2)$$

**Generalizing by Induction:**
The formula for the state at any time step $k$ is the sum of the **Free Response** (due to initial conditions) and the **Forced Response** (due to inputs).

**Final Formula:**  

$$
x(k) = \underbrace{A\_d^k x(0)}_{\text{Free Response}} + \underbrace{\sum_{j=0}^{k-1} A\_d^{k-1-j} B\_d u(j)}_{\text{Forced Response}} = x\_{zi}(k) + x\_{zs}(k)
$$

If we multiply it by $C\_d$ matrix, we get the corresponding output:


$$
y(k) = \underbrace{C\_d A\_d^k x(0)}_{\text{Free Response}} + \underbrace{C\_d \sum_{j=0}^{k-1} A\_d^{k-1-j} B\_d u(j)}_{\text{Forced Response}} = y\_{zi}(k) + y\_{zs}(k)
$$

These formulations allows us to compute **samples**. However, we usually prefer to handle with **functions**. Let's introduce **$\mathcal{Z}$-transform**

---
WIPPPPPP
