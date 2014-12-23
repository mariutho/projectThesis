%% Load data
clc; clear; clear kalman_r;

constants = huginConstants;
KF_testScript;
clc; clear; clear kalman_r;
navlabPath = 'C:/Users/mariutho/NavLab/Download/sim_ini/basic/';
simPath = 'C:/Users/mariutho/prosjektoppgave_sim/simData/';
dvlPath = [ simPath '../dvlAiding'];

cd(dvlPath)

load('sim_13-Dec-2014_15-35-34')
%load('sim_09-dec-2014_17-54-45');
%load('sim_26-Nov-2014_14-40-18')

%% Create persistent variables
t_stop = 600;
ind = 1+(0:t_stop*10);
time = (ind-1)/10;

% True state
Lat = sav_latitude(ind);
Lon = sav_longitude(ind);
v = sav_v_EB_L(1:2,ind);
yaw = sav_yaw(ind);
x_true.signals.values = [yaw; v; Lat; Lon]';
x_true.time = time';

% x_check
Lat_c = sav_latitude_naveq_c(ind);
Lon_c = sav_longitude_naveq_c(ind);
v_c = sav_v_EB_L_naveq_c(1:2,ind);
yaw_c = sav_yaw_naveq_c(ind);
for i = ind
	if yaw_c(i) > pi
		yaw_c(i) = yaw_c(i) - 2*pi;
	end
end
x_check.signals.values = [yaw_c; v_c; Lat_c; Lon_c]';
x_check.time = time';

% Measured force
f.signals.values = sav_f_IB_B_a(1:2,ind)';
f.time = time';

% Intialize variables
sigma_dvl = 4e-3;
var_dvl = sigma_dvl^2;
E = E_D();
E = E(:,1:2);

use_dvl_bias = 1;
T_dvl = 600;
sigma_dvl_rw = 1e-3;
var_dvl_rw = sigma_dvl_rw^2;

% Initialize 
G = inv(E'*E)*E';

%% Create per run variables
dvl_wn.time = time;
dvl_wn.signals.values = normrnd(0, sigma_dvl, t_stop/0.1+1, 4);

dvl_rw.time = time;
dvl_rw.signals.values = normrnd(0, sigma_dvl_rw, t_stop/0.1+1, 4);


% Create dvl-noise


%% Do simulation 
sim dvlAiding;

%% Monte Carlo - Coming soon
% kalman_LS(u(1:2),u(3:7),u(8:9))

%% Get results
simSize = length(x_hat.signals.values(:,1));
estTime = (0:(simSize-1))/10;

% pos
Lat_e = x_hat.signals.values(:,4)';
Lon_e = x_hat.signals.values(:,5)';
Lat_bar = Lat_c(1:simSize) - Lat_e;
Lon_bar = Lon_c(1:simSize) - Lon_e;
 
% vel
u_e = x_hat.signals.values(:,2)';
v_e = x_hat.signals.values(:,3)';
u_bar = v_c(1,1:simSize) - u_e;
v_bar = v_c(2,1:simSize) - v_e;

% vel LS
u_e_LS = x_hat_LS.signals.values(:,2)';
v_e_LS = x_hat_LS.signals.values(:,3)';
u_bar_LS = v_c(1,1:simSize) - u_e_LS;
v_bar_LS = v_c(2,1:simSize) - v_e_LS;

%% Integrate v
T_sample = 0.1;
p_N = T_sample*cumtrapz(u_bar);
p_E = T_sample*cumtrapz(v_bar);
p_MB_M_e = [p_N; p_E];

p_N_LS = T_sample*cumtrapz(u_bar_LS);
p_E_LS = T_sample*cumtrapz(v_bar_LS);

disp('done');

%% Plots
sim_plots;
