function x_out = kalman_r(dz, x_check, f)

	assert(length(dz) == 4);
	assert(length(x_check) == 5);
	assert(length(f) == 2);
	N = length(x_check) + 3;
	% state vector
	% [dpsi, dv, dp, b_a, b_g]'
	
	var_dvl = (4e-3)^2;
	r = var_dvl;
	var_a = 1.47e-4;
	var_g = 1.75e-6;
	var_ba = 9.81e-4;
	var_bg = 4.85e-8;
	vars = [var_g; var_a; var_a; 1e-15; 1e-15; var_ba; var_ba; var_bg];
	
    persistent P x_est Q R x_pred k
	% Initialization
    if isempty(P)
		% Variances
		k = 1;
		
        % First time through the code do initialization
        x_est = zeros(N,1);
		P = zeros(N);
        Q = diag(vars);
        R = var_dvl*eye(4);
	end

    % Propagate the state estimate and covariance matrix:
	% Time update
	tau = 0.1;
	F_r = ErrorPropReduced(x_check, f, tau);
  	x_pred = F_r*x_est;
 	P = F_r*P*F_r' + Q;
	
	% Watch this
	x_pred(1) = 0;
	x_est = x_pred;
	
	
	% Measurment update every second
	if mod(k,10) == 0
		% Find innovation and covariance
		s = dz - h(x_check, x_pred);
		[H,H_b] = H_lin_r(x_check, x_pred);
		S = H*P*H' + R;
		
		% Error detection on beams
		sensors = ones(4,1);
		%gamma = 5*sqrt(2); % 5 sigma
		gamma = 100000;
		for countI = 1:4
			epsilon(countI) = s(countI)^2/S(countI,countI);
			if epsilon(countI) > gamma
				sensors(countI) = 0;
			end	
		end
		
		% Turn on error detection
		errorDetection = 1;
		if ~errorDetection
			sensors = [1;1;1;1];
		end
		
		% Turn on determined beam configuration
		beamConfig = 0;
		if beamConfig == 1
			sensors = [1;0;0;0];
		end
		
		N_sensors = sum(sensors);
		% If no valid measurements, skip update
		if N_sensors > 2 || isequal(sensors,[1;0;0;1]) || isequal(sensors,[0;1;1;0]) || isequal(sensors,[1;1;0;0]) || isequal(sensors,[0;0;1;1]) || beamConfig
			% Filter out bad measurements and create new E-matrix
			E = E_D_r();
			E_new = [];
			s_new = [];
			for countJ = 1:4
				if sensors(countJ)
					E_new = [E_new; E(countJ,:)];
					s_new = [s_new; s(countJ)];
				end
			end

			% Create new measurement matrix
			R_new = r*eye(N_sensors);
			H_new = E_new*H_b;
			S_new = H_new*P*H_new' + R_new;

			% Calculate the new Kalman gain
			K = P*H_new'*inv(S_new);

			% Update estimate and covariances
			x_est = x_pred + K*s_new;
			P = (eye(N)-K*H_new)*P;

			% Watch this
			x_est(1) = 0;
		end
	end
	
	% Counter
	x_out = [x_est; x_pred];
	k = k+1;
	if mod(k,1000) == 0
		k
	end
end

function [H,H_b_r] = H_lin_r(x_check, x_pred)
	x_check_aug = [0;0;x_check(1:3); 0; x_check(4:5); zeros(7,1)];
	x_pred_aug = [0;0;x_pred(1:3); 0; x_pred(4:5); 0; x_pred(6:7); 0; 0; 0; x_pred(8)];
	[~, H_b] = H_lin(x_check_aug, x_pred_aug);
	H_b_r = H_red(H_b);
	H = E_D_r()*H_b_r;
end

function h_out = h(x_check_r, dx_r) 
	% x_full = [0;0; x_check_r(1:3); 0];
	% dx_full = [0;0; dx_r(1:3); 0];
	x_full = [0;0; x_check_r(1:3); 0];
	dx_full = [0;0; dx_r(1:3); 0];
	h_out = measurement(x_full, dx_full);
end

function E = E_D_r()
	beta = -60*pi/180;
	alpha = pi/4 + pi/2*(0:3)';
	E = [cos(beta)*cos(alpha) cos(beta)*sin(alpha)];
end

function H_r = H_red(H_b)
	H_r = [H_b(1:2,3:5) zeros(2,5)];
end