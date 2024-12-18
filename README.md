# Linear MPC in MATLAB/Simulink using YALMIP
This example shows how to formulate a linear MPC problem using [YALMIP](https://yalmip.github.io/). The simulation can be carried out in either MATLAB or Simulink.

A linear MPC formulation usually has a quadratic objective, linear system dynamics and simple bounds on states or inputs:

$$
\begin{aligned}
  \min_{x_{0:N}, u_{0:N-1}} \quad & (x_N-x_r)^\top Q_N(x_N-x_r) + \sum_{k=0}^{N-1}{ (x_k-x_r)^\top Q (x_k-x_r) + u_k^\top R u_k } \\
  \textrm{subject to} \quad & x_{k+1} = A x_k + B u_k\\
                    & x_0 = \bar{x} \\
                    & x_{\text{min}} \leq x_k \leq x_{\text{max}} \\
                    & u_{\text{min}} \leq u_k \leq u_{\text{max}}
\end{aligned}
$$

Note: the simulation and MPC parameters are not necessarily realistic, they were chosen to be as simple as possible.
