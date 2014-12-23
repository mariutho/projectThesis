projectThesis
=============
This project includes the code for all simulations done with the project report "Tight integration of DVL beam velocities in INS for autonomous underwater vehicles".

The true trajectories, simulated IMU measurements and an INS estimate calculated using only the navigation equations is generated with the navigation software NavLab, which is not included in this project. However, 100 realizations of an example path for the HUGIN is included here. EKF estimates can be calculated using "runMultiSims.m", and are stored in the desired folder. The file "analyzeSims.m" loads the estimates data and combines to updated state estimates. Plots can be generated with the script "plotEstimates.m".