## MPC-CBF-ACC2021
We propose a control framework which unifies the model predictive control and control barrier functions, where terminal cost function serves as control Lyapunov functions for stability. This is the reference implementation of our paper:

If you find this project useful in your work, please consider citing following work:
```
@inproceedings{zeng2021mpccbf,
  title={Safety-critical model predictive control with discrete-time control barrier function},
  author={Zeng, Jun and Zhang, Bike and Sreenath, Koushil},
  booktitle={2021 American Control Conference (ACC)},
  year={2021}
}
```

### Instructions
The 2D double integrator is assigned to reach the target position at origin while avoiding obstacles. We have three classes for different controllers: `DCLFDCBF.m` (DCLF-DCBF), `MPCCBF.m` (MPC-CBF) and `MPCDC` (MPC-DC), respectively.

Moreover, to illustrate the performance among them, we have:
* `test.m`: Run DCLF-DCBF/MPC-CBF/MPC-DC respectively.
* `testGamma.m`: Run analysis for different hyperparameter $\gamma$.
* `testHorizon.m`: Run analysis for different horizon.
* `testBenchmark.m`: Run analysis for some benchmark.