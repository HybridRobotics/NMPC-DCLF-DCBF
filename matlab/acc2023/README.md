# Iterative-MPC-DHOCBF
Matlab code for the paper *"Iterative Convex Optimization for Model Predictive Control with Discrete-Time High-Order Control Barrier Functions"*, accepted by IEEE American Control Conference (ACC) 2023, Authors: *Shuo Liu, Jun Zeng,  Koushil Sreenath and Calin Belta* [[PDF]](https://arxiv.org/pdf/2210.04361.pdf)

In this paper, we propose a framework that solves the safety critical MPC problem in an iterative optimization, which is applicable for any relative-degree control barrier functions. In the proposed formulation, the nonlinear system dynamics as well as the safety constraints modeled as discrete-time high order control barrier functions (DHOCBF) are linearized at each time step. Our formulation is generally valid for any control barrier function with an arbitrary relative-degree. The advantages of fast computational performance with safety guarantee are analyzed and validated with numerical results.

## Citing
If you find this project useful in your work, please consider citing following work:
```
@inproceedings{liu2023iterative,
  title={Iterative Convex Optimization for Model Predictive Control with Discrete-Time High-Order Control Barrier Functions},
  author={Liu, Shuo and Zeng, Jun and Sreenath, Koushil and Belta, Calin A},
  booktitle={2023 American Control Conference (ACC)},
  year={2023}
}
```

## Instruction
There are two subfolders `closedloop_performance` and `benchmark`. `closedloop_performance` contains all information to generate figure 2, 3, 4 in paper and `benchmark` conatins code to generate data for table 1,2 in paper.

## Subfolder `closedloop_performance`

### Code Descriptions
* `Closedloop_Trajectories_Hyperparameters.m` includes codes to generate necessary data for figure 2-a and figure 3; *"Iterative_Convergence"* includes codes to generate necessary data for figure 2-b,c,d.
* `Maximum_Iterations.m` includes codes to generate necessary data for figure 4.   
* `NMPCDCBF1.m` and `NMPCDCBF2.m` include codes related to NMPC-DHOCBF with `mcbf=1` and `mcbf=2` respectively and `FigureGenerate.m` includes codes to transfer the data into figures in paper.

### Usage
* Run `Closedloop_Trajectories_Hyperparameters.m`, `Iterative_Convergence.m` and `Maximum_Iterations.m` and the data files are generated as MATLAB mat files.  
* Run `FigureGenerate.m` to generate all figures in paper, which will be saved in folder `closedloop_performance/figures`.

## Subfolder `benchmark`

### Code Descriptions
There are four folders including the codes to generate necessary dada for table 1,2 in paper.   
* `gamma1_0p4gamma2_0p4` includes the codes for iMPC-DHOCBF and NMPC-DHOCBF with decay rate parameters `gamma1=0.4, gamma2=0.4, mcbf=2`.
* `gamma1_0p4` includes the codes for iMPC-DHOCBF and NMPC-DHOCBF with decay rate parameters `gamma1=0.4, mcbf=1`.
* `gamma1_0p6gamma2_0p6` includes the codes for iMPC-DHOCBF and NMPC-DHOCBF with decay rate parameters `gamma1=0.6, gamma2=0.6, mcbf=2`.
* `gamma1_0p6` includes the codes for iMPC-DHOCBF and NMPC-DHOCBF with decay rate parameters `gamma1=0.6, mcbf=1`.
### Usage
* Run `InitialState.m` first to generate 1000 random initial states in the mat file `InitialStateData.mat`, then copy it to each folder.
* In each folder, run the file `test_comprehensive.m` and you will see corresponding data files generated as MATLAB mat files `feasibility_N.mat` and `timecom_N.mat` which includes the information about infeasible rate and mean/variance of computing time (stored in matrices `nmpcdata` and `impcdata`) in paper from generating one time-step trajectories for iMPC-DHOCBF and NMPC-DHOCBF.

### Warnings
* It may take a long time to run the file `test_comprehensive.m` (several hours to a day depending on your computer when the number of horizon reaches large). Instead, if you want to run the file for different number of horizon parallelly to speed up the running process, in folder `test_each_horizon.m`, there are six files called `test_N4.m`, `test_N8.m`, `test_N12.m`, `test_N16.m`, `test_N20.m`,`test_N24.m` which correspond to the number of horizon 4, 8, 12, 16, 20, 24 for iMPC-DHOCBF and NMPC-DHOCBF. By copying `InitialStateData.mat` to this folder then you can run these 6 files parallely, in order to generate same MATLAB mat files `feasibility_N.mat` and `timecom_N.mat` which  includes the information about infeasible rate and mean/variance of computing time in paper from generating one time-step trajectories for iMPC-DHOCBF and NMPC-DHOCBF. You should run the file `tabledata.m` next and all useful data in table are stored in matrices `nmpcdata` and `impcdata`.

## Post Analysis
1. For comparison of capability of generating safe optimal trajectories for different numbers of horizon and hyperparameters, please see the figure below:
![image](https://github.com/ShockLeo/Iterative-MPC-DHOCBF/blob/main/closedloop_performance/performance2.png)
2. For comparison of infeasible rate and mean/variance of computing time from generating one time-step trajectories, please see the figure below:
![image](https://github.com/ShockLeo/Iterative-MPC-DHOCBF/blob/main/benchmark/performance1.png) 
For other figures please refer to the paper.

## Dependencies
The packages needed for running the code are [Yalmip](https://yalmip.github.io/) and [IPOPT](https://github.com/coin-or/Ipopt) for NMPC-DHOCBF and [OSQP](https://github.com/osqp/osqp) for iMPC-DHOCBF.
