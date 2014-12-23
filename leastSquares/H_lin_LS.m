function H_out = H_lin_LS(x_check, x_pred)
% 	x_check_aug = [0;0;x_check(1:3); 0; x_check(4:5); zeros(7,1)];
% 	x_pred_aug = [0;0;x_pred(1:3); 0; x_pred(4:5); 0; x_pred(6:7); 0; 0; 0; x_pred(8)];
% 	H = H_lin_LS_full(x_check_aug, x_pred_aug);
% 	H_out = [H(1:2,3:5) zeros(2,5)];

	% 
	H_out = zeros(2,8);
	H_out(1,2) = -1;
	H_out(2,3) = -1;
end

function H = H_lin_LS_full(x, dx)
	v = x(4:6);
	dTheta = dx(1:3);
	dv = dx(4:6);
	
	H_Theta = -skew(v - dv);
	H_v = -(eye(3) + skew(dTheta));
	z_3 = zeros(3);
	H = [H_Theta H_v z_3 z_3 z_3];
end