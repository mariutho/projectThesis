function Phi = ErrorPropReduced(x, f_i, tau)
	% 3 DOF checks
	assert(length(x) == 5);
	assert(length(f_i) == 2);
	
	% Augment to get 6 DOF
	Theta = [0;0;x(1)];
	v = [x(2:3); 0];
	p = [x(4:5); 0];
	x_aug = [Theta; v; p];
	f_aug = [f_i; 0];
	
	F_full = ErrorProp(x_aug, f_aug, tau);
	
	F_11 = F_full(3:5,3:5);
	F_12 = F_full(3:5,7:8);
	F_13 = F_full(3:5,10:11);
	F_14 = F_full(3:5,15);
	F_21 = F_full(7:8,3:5);
	F_22 = F_full(7:8,7:8);
	F_23 = F_full(7:8,10:11);
	F_33 = F_full(10:11,10:11);
	F_44 = F_full(15,15);

	F_1 = [F_11	F_12	F_13	F_14];
	F_2 = [F_21	F_22	F_23	zeros(2,1)];
	F_3 = [zeros(2,5)	F_33	zeros(2,1)];
	F_4 = [zeros(1,7)	F_44];
	Phi = [F_1; F_2; F_3; F_4];
end