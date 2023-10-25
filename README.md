# Review on various controller for optimal trajectory tracking.

## Introduction
- This repository contains experimental scripts that implement different feedback-loop on quadcopter for optimal trajectory control and comparison of those controllers based on how accurately & efficient they allowed the quadcopter to follow the given path or trajectory. 

- The experiment is performed in two different frameworks, first, 
	- ROS-Gazebo: An open-source simulator called Gazebo, which is a 3D dynamic multi-robot environment capable of recreating the complex worlds that would be encountered by the next generation of mobile robots [1](https://ieeexplore.ieee.org/abstract/document/1389727), where a the quadcopter model, derived from RotorS, a modular Micro Aerial Vehicle (MAV) simulation stack, which enables a quick start to perform research on MAVs [2](https://link.springer.com/chapter/10.1007/978-3-319-26054-9_23). 
	> Note: The ROS-Gazebo Model used is a forked from [erts-RnD/survey_and_rescue](https://github.com/erts-RnD/survey_and_rescue) & [erts-RnD/sentinel_drone](https://github.com/erts-RnD/sentinel_drone) repositories. 

	- And second on Matlab-Simulink, for quick implementation of State-Space controllers like LQR.


## Methodology

- The review on performance of different trajectory tracking controller are...
	1. **Proportional Integral Controller** (*PID*) 
		1. Using Ziegler–Nichols method [3]
		2. Plant Tuning-based design method 
		3. ITAE Tuning-Based Design Method Implemented [4]

	2. **Linear–Quadratic Regulator** *(LQR)* controller

	3. **Cascaded Controllers**
		1. LQR-PI
		2. LQR-PID
  		3. Observer-based LQR
 
## Contributions 
- Connor Reece: Simulink models for the algorithms and MALTAB code for PID
- Vishal Gupta: ROS Gazebo implementations and set up
- Kaushik Iyer: LQR (and variants) codes in MATLAB and Lyapunov analysis

## Results

- The intended performance criteria in based on *Lyapunov Stability Analysis* 

- The ways to quantify the analysis are
	1. Perturbing the quadcopter at its stable position & orientation (Pose) and observing the responses, essentially the **step response analysis**.

	2. And extending the step response analysis to a set of waypoints, sampled from the 3D trajectory equation, at very small time instances to make the drone appear to track that trajectory. 

> * Team Project Report - [EEE587-Project-Report](./EEE587-Project-Report.pdf)
>
> * Presentation - [Quadcopter Optimal Trajectory Control: A Comparative Study](https://onedrive.live.com/view.aspx?resid=6A66E77367076643!18125&ithint=file%2cpptx&authkey=!APm0nHRP8eKRLjM)

## References

[1] Koenig, N., & Howard, A. (2004, September). Design and use paradigms for gazebo, an open-source multi-robot simulator. In 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)(IEEE Cat. No. 04CH37566) (Vol. 3, pp. 2149-2154). IEEE.

[2] Furrer, F., Burri, M., Achtelik, M., & Siegwart, R. (2016). Rotors—a modular gazebo mav simulator framework. Robot Operating System (ROS) The Complete Reference (Volume 1), 595-625.

[3] Åström, K. J., & Hägglund, T. (2004). Revisiting the Ziegler–Nichols step response method for PID control. Journal of process control, 14(6), 635-650.

[4] Argentim, L. M., Rezende, W. C., Santos, P. E., & Aguiar, R. A. (2013, May). PID, LQR and LQR-PID on a quadcopter platform. In 2013 International Conference on Informatics, Electronics and Vision (ICIEV) (pp. 1-6). IEEE.



