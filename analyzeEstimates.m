N = 100;

estPath = 'simData/x_hat/';
pathPath = 'simData/pathNaveqData/';

cd(simPath)


pathFileRoot = 'mcSim_test_';
t_stop = 600;
ind = 1+(0:t_stop*10);
time = (ind-1)/10;



clear posError posErrorLS posErrorC
A = 1;
B = 100;
for i = A:B
	i
	% Load path data
	simNumString = num2str(i);
	pathFileRoot = 'mcSim_test_';
	pathFile = [pathPath pathFileRoot simNumString];
	path = load(pathFile);
	
	% Load estimate data
	estFileRoot = 'x_est_';
	estFile = [estPath estFileRoot simNumString];
	est = load(estFile);
	
	% True state
	Lat = path.sav_latitude(ind);
	Lon = path.sav_longitude(ind);
	v = path.sav_v_EB_L(1:2,ind);
	yaw = path.sav_yaw(ind);
	p_M = path.sav_p_MB_M(1:2,ind);

	v_c = path.sav_v_EB_L_naveq_c(1:2,ind);
	p_M_c = path.sav_p_MB_M_naveq_c(1:2,ind);
	
	u_e = est.x_hat.signals.values(:,2)';
	v_e = est.x_hat.signals.values(:,3)';
	u_bar = v_c(1,:) - u_e;
	v_bar = v_c(2,:) - v_e;

	u_e_LS = est.x_hat_LS.signals.values(:,2)';
	v_e_LS = est.x_hat_LS.signals.values(:,3)';
	u_bar_LS = v_c(1,:) - u_e_LS;
	v_bar_LS = v_c(2,:) - v_e_LS;

	T_sample = 0.1;
	p_N = T_sample*cumtrapz(u_bar);
	p_E = T_sample*cumtrapz(v_bar);
	p_M_e = [p_N; p_E];

	p_N_LS = T_sample*cumtrapz(u_bar_LS);
	p_E_LS = T_sample*cumtrapz(v_bar_LS);
	p_M_LS = [p_N_LS; p_E_LS];
	
	for j = ind
		posError(i,j) = norm(p_M(:,j) - p_M_e(:,j));
		posErrorLS(i,j) = norm(p_M(:,j) - p_M_LS(:,j));
		posErrorC(i,j) = norm(p_M(:,j) - p_M_c(:,j));
	end
	
	% calculate errors
end

meanPosError = mean(posError(A:B,:));
meanPosErrorLS = mean(posErrorLS(A:B,:));
meanPosErrorC = mean(posErrorC(A:B,:));


disp('done')