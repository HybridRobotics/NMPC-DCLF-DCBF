## NMPC-DCLF-DCBF-CDC2021
In this paper, we discuss the feasibility, safety and complexity about existing optimal control laws using control barrier functions. This is the reference implementation of our paper.

If you find this code useful in your work, please consider citing:
```
@inproceedings{zeng2021nmpc-dclf-dcbf,
  title={Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with Discrete-Time Control Barrier Functions},
  author={Zeng, Jun and Li, Zhongyu and Sreenath, Koushil},
  booktitle={2021 IEEE Conference on Decision and Control (CDC)},
  year={2021}
}
```

### Instructions
Several test files are created to reproduce the results in the paper:
* `testFeasibilityMPCDCBF.m` compares the feasibility performance between MPC-DCBF and NMPC-DCBF with different hyperparameters.
* `testFeasibilityMPCGCBF.m` compares the feasibility performance between MPC-GCBF and NMPC-DCBF with different hyperparameters.
* `testFeasibilityDCLFDCBF.m` compares the feasibility performance between DCLF-DCBF and NMPC-DCLF-DCBF.
* `testSafety.m` compares the safety performance between MPC-CBF, MPC-GCBF and NMPC-DCBF.

Notice that running either `testFeasibilityMPCDCBF.m` or `testFeasibilityMPCGCBF.m` or `testFeasibilityDCLFDCBF.m` will take more than 4 hours due to the weak memory management using matlab together with excessive Yalmip calling. To check results or have a quick visualization, feel free to load data directly from the `/data`.