function x_out = kalman_LS(z_dvl, z_ins, x_check, f)
	assert(length(z_dvl) == 2);
	assert(length(z_ins) == 2);
	assert(length(x_check) == 5);
	assert(length(f) == 2);
	N = length(x_check) + 3;
	
	% state vector
	% [dpsi, dv, dp, b_a, b_g]'
	
	%var_dvl = (4e-3)^2;
	var_dvl = (6e-3)^2;
	var_a = 1.47e-4;
	var_g = 1.75e-6;
	var_ba = 9.81e-4;
	var_bg = 4.85e-8;
	vars = [var_g; var_a; var_a; 1e-15; 1e-15; var_ba; var_ba; var_bg];
	
	
    persistent P_LS x_est_LS Q_LS R_LS x_pred_LS k_LS
	% Initialization
	if isempty(P_LS)
		% Variances
		k_LS = 1;
		
        % First time through the code do initialization
        x_est_LS = zeros(N,1);
		P_LS = zeros(N);
        Q_LS = diag(vars);
        R_LS = var_dvl*eye(2);
	end

    % Propagate the state estimate and covariance matrix:
	% Time update
	tau = 0.1;
	F_r = ErrorPropReduced(x_check, f, tau);
  	x_pred_LS = F_r*x_est_LS;
 	P_LS = F_r*P_LS*F_r' + Q_LS;
	
	% Watch this
	x_pred_LS(1) = 0;
	x_est_LS = x_pred_LS;
	
	% Measurment update every second
	if mod(k_LS,10) == 0
		R_bl = R_bl2(x_check(1));
		dz = R_bl*z_dvl - z_ins;
		% Find innovation and covariance
		s = dz - h_x_LS(x_check, x_pred_LS);
		H = H_lin_LS(x_check, x_pred_LS);
		S = H*P_LS*H' + R_LS;
		
		
		errorDetection = 1;
		
		gamma = 10; % 5 sigma
		% If no valid measurements, skip update
		if s'*inv(S)*s < gamma || ~errorDetection
			% Calculate the new Kalman gain
			K = P_LS*H'*inv(S);
			% Update estimate and covariance
			x_est_LS = x_pred_LS + K*s;
			P_LS = (eye(N)-K*H)*P_LS;

			% Watch this
			x_est_LS(1) = 0;
		end
	end
	x_out = [x_est_LS; x_pred_LS];
	
	% Counter
	k_LS = k_LS+1;
end

function R = R_bl2(psi)
	R = [cos(psi)	-sin(psi);
		sin(psi)	cos(psi)];
end