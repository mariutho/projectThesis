%% Setup path data
clc; clear; clear kalman_r; clear kalman_LS;

KF_testScript;
clc; clear; clear kalman_r; clear kalman_LS

N = 100;

for i = 1:N
	simFileRoot = 'simData/pathNaveqData/mcSim_test_';
	t_stop = 600;
	ind = 1+(0:t_stop*10);
	time = (ind-1)/10;

	% Intialize variables
	E = E_D();
	E = E(:,1:2);
	G = inv(E'*E)*E';

	sigma_dvl = 4e-3;
	var_dvl = sigma_dvl^2;
	use_dvl_bias = 1;
	T_dvl = 600;
	sigma_dvl_rw = 1e-3;
	var_dvl_rw = sigma_dvl_rw^2;
	
	i
	% Load data
	simNumString = num2str(i);
	simFile = [simFileRoot simNumString];
	
	path = load(simFile);
	
	
	% True state
	Lat = path.sav_latitude(ind);
	Lon = path.sav_longitude(ind);
	v = path.sav_v_EB_L(1:2,ind);
	yaw = path.sav_yaw(ind);
	x_true.signals.values = [yaw; v; Lat; Lon]';
	x_true.time = time';

	% x_check
	Lat_c = path.sav_latitude_naveq_c(ind);
	Lon_c = path.sav_longitude_naveq_c(ind);
	v_c = path.sav_v_EB_L_naveq_c(1:2,ind);
	yaw_c = path.sav_yaw_naveq_c(ind);
	for i = ind
		if yaw_c(i) > pi
			yaw_c(i) = yaw_c(i) - 2*pi;
		end
	end
	x_check.signals.values = [yaw_c; v_c; Lat_c; Lon_c]';
	x_check.time = time';

	% Measured force
	f.signals.values = path.sav_f_IB_B_a(1:2,ind)';
	f.time = time';

	% DVL measurements
	dvl_wn.time = time;
	dvl_wn.signals.values = normrnd(0, sigma_dvl, t_stop/0.1+1, 4);

	dvl_rw.time = time;
	dvl_rw.signals.values = normrnd(0, sigma_dvl_rw, t_stop/0.1+1, 4);

	
	% DVL dropout
	errorRate = 0.10;
	decisionVar = unifrnd(0, 1, ind(end), 4);
	failValues = zeros(ind(end),4);
	for k = ind
		for l = 1:4
			if decisionVar(k,l) < errorRate
				failValues(k,l) = unifrnd(-1e9,1e9);
			end
		end
	end
	
	dvlFaults.time = time;
	dvlFaults.signals.values = failValues;
	
	
	% Simulation
	sim dvlAiding;
	
	saveName = 'simData/x_hat/x_hat_';
	saveFull = [saveName simNumString]
	save(saveFull, 'x_hat', 'x_hat_LS')
	
	clear all; clear kalman_r; clear kalman_LS;
end

disp('done')