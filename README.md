**Status**: The implementation code for corresponding papers will be merged here and new papers will be added in an inverse order of submission.

### Introduction

In this repository, a collection of our work is presented where nonlinear model predictive control (NMPC) with control Lyapunov functions (CLFs) and control barrier functions (CBFs) are applied.

### Dependencies
The packages needed for running the code are [Yalmip](https://yalmip.github.io/) and [IPOPT](https://projects.coin-or.org/Ipopt/wiki/MatlabInterface).

We also provide the zipped version of precompiled .mex files for IPOPT in the folder `packages` in case you don't have it. Unzip that file and add those .mex files into your MATLAB path.

### Citing

#### Theoretical Publications

If you find this project useful in your work, please consider citing following work:

* A. Thirugnanam, J. Zeng, K. Sreenath. "Safety-Critical Control and Planning for Obstacle Avoidance between Polytopes with Control Barrier Functions." *2022 IEEE International Conference on Robotics and Automation (ICRA)*. [[IEEE]](https://ieeexplore.ieee.org/document/9812334) [[arXiv]](https://arxiv.org/abs/2109.12313) [[Video]](https://youtu.be/wucophROPRY) [[BibTex]](bibtex/icra2022_nmpc_dcbf_polytope.md)

* J. Zeng, Z. Li, K. Sreenath. "Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with Discrete-Time Control Barrier Functions." *2021 IEEE Conference on Decision and Control (CDC)*. [[IEEE]](https://ieeexplore.ieee.org/document/9683174) [[arXiv]](https://arxiv.org/abs/2105.10596) [[Docs]](matlab/cdc2021/README.md) [[Code]](matlab/cdc2021) [[BibTex]](bibtex/cdc2022_nmpc_dcbf_feasibility.md)

* J. Zeng, B. Zhang and K. Sreenath. "Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function." *2021 IEEE American Control Conference (ACC)*. [[IEEE]](https://ieeexplore.ieee.org/document/9483029) [[arXiv]](https://arxiv.org/abs/2007.11718) [[Docs]](matlab/acc2021/README.md) [[Code]](matlab/acc2021) [[BibTex]](bibtex/acc2021_nmpc_dcbf.md)

#### Applicational Publications

* Z. Li, J. Zeng, A. Thirugnanam, K. Sreenath. "Bridging Model-based Safety and Model-free Reinforcement Learning through System Identification of Low Dimensional Linear Models." *2022 Proceedings of Robotics: Science and Systems (RSS)*. [[RSS]](http://www.roboticsproceedings.org/rss18/p033.html) [[arXiv]](https://arxiv.org/abs/2205.05787) [[BibTex]](bibtex/rss2022_nmpc_dcbf_legged_robots.md) [[Webpage]](https://sites.google.com/berkeley.edu/rl-sysid-rss2022/home)