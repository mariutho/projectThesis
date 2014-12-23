%% 6 DOF
dz = [ 0.1; -0.1; -0.1; 0.1];
Theta = [0;0;0];
v = [2.3; 0.1; 0];
p = [0;0;0];
b_a = normrnd(0,0.004,3,1);
b_g = normrnd(0,0.001,3,1);
x_check = zeros(9,1);
clear;

%% 3 DOF
dz_r = sin(pi/6)*[ 0.01; -0.01; -0.01; 0.01];
psi_r = 0;
v_r = [2.04; 0.01];
p_r = [0;0];
b_a_r = normrnd(0,0.004,2,1);
b_g_r = normrnd(0,0.001);
f_r = [0.1; 0.05];

x_check_r = [psi_r; v_r; p_r];

%% Simulation
dx_bar_r = kalman_r(dz_r, x_check_r, f_r);
clear kalman_r;
x_bar_r = x_check_r + dx_bar_r(1:5);

%% LS
E = E_D();
E = E(:,1:2);

G = inv(E'*E)*E';
z_DVL = [2.001; -0.001];
z_INS = [2.04; 0.01];
dx_LS = kalman_LS(z_DVL, z_INS, x_check_r, f_r);

clear kalman_LS;