# Flight Trajectory Generation through a Collocation Approach with Successive Linear Programming
This project develops a collocation framework integrated with `successive linear programming (SLP)` to address non-convex trajectory generation problems. 
![image](https://github.com/lenleo1/Colloc_SLP/blob/master/docs/figures/trap_collocation.jpg)
- A linear programming subproblem is constructed by linearizing nonlinear collocation constraints and path constraints. This subproblem aims to find optimal increments of parameters, states, and controls to refine a reference trajectory, which is subsequently re-linearized to formulate subsequent subproblems.
![image](https://github.com/lenleo1/Colloc_SLP/blob/master/docs/figures/grid_demo.jpg)
- To address the potential `unboundedness` and `infeasibility` of the subproblem, linearized constraints are dealt with via exact penalties, and trust regions are enforced at each collocation node. The sizes of these trust regions are penalized using a small weight.
![image](https://github.com/lenleo1/Colloc_SLP/blob/master/docs/figures/multi_trust_regions.jpg)
- The maximum allowable trust region size is dynamically adjusted based on the linearization error to assure global convergence.
- Practical applications on fixed-wing aircraft, quadrotors, and other dynamic systems are included.
