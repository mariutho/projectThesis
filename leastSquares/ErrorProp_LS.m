function Phi_ins = ErrorProp(x, f_i, tau)
	% 6 DOF checks
	assert(length(x) == 9);
	assert(length(f_i) == 3);

	%	[dpsi, dv, dp, b_a, b_g]'
	z_3 = zeros(3);
	R_bn = eulerRot(x);
	T_1 = 1800;
	T_2 = 1800;
	T_a = -1/T_1*eye(3);
	T_g = -1/T_2*eye(3);
	
	%Original
	F_ins = [
		F_11(x)		F_12(x)	F_13(x)	z_3		R_bn;
		F_21(x,f_i)	F_22(x)	F_23(x)	R_bn	z_3;
		z_3			F_32(x)	F_33(x)	z_3		z_3;
		z_3			z_3		z_3		T_a		z_3;
		z_3			z_3		z_3		z_3		T_g];
	
% Problem in F_32
	
	Phi_ins = eye(15) + F_ins*tau;
end

function test = testFun()
	x = ones(15,1);
	f = [1;2;3];
	R_E_1 = R_E(1);
	R_N_1 = R_N(1);
	F_11_x = F_11(x);
	F_12_x = F_12(x);
	F_13_x = F_13(x);
	F_21_x = F_21(x,f);
	F_22_x = F_22(x);
	F_23_x = F_23(x);
	F_32_x = F_32(x);
	F_33_x = F_33(x);
	R = eulerRot(x(4:6));
end

function F = F_11(x)
	% http://hpiers.obspm.fr/eop-pc/models/constants.html
	omega_ie = 7.292e-5;
	v_N = x(4);
	v_E = x(5);
	L_b = x(7);
	h_b = x(9);
	
	omega_ie_n = omega_ie*[cos(L_b);	0;	-sin(L_b)];
	omega_en_n = [
		v_E/(R_E(L_b) + h_b);
		-v_N/(R_N(L_b) + h_b);
		-v_E*tan(L_b)/(R_E(L_b) - h_b)
		];
	omega_in_n = omega_en_n + omega_ie_n;
	
	F = skew(omega_in_n);
end

function F = F_12(x)
	L_b = x(7);
	h_b = x(9);
	F = zeros(3,3);
	F(1,2) = -1/(R_E(L_b) + h_b);
	F(2,1) = 1/(R_N(L_b) + h_b);
	F(3,2) = tan(L_b)/(R_E(L_b) + h_b);
end

function F = F_13(x)
	omega_ie = 7.292e-5;
	v_N = x(4);
	v_E = x(5);
	L_b = x(7);
	h_b = x(9);
	
	F = [
		omega_ie*sin(L_b)	0	v_E/(R_E(L_b) + h_b)^2;
		0					0	-v_N/(R_N(L_b) + h_b)^2;
		omega_ie*cos(L_b) + v_E/((R_E(L_b) + h_b)*(cos(L_b)^2))	0	-v_E*tan(L_b)/(R_E(L_b) + h_b)^2
		];
end

function F = F_21(x,f_i) 
	R = eulerRot(x(4:6));
	F = skew(R*f_i);
end

function F = F_22(x)
	omega_ie = 7.292e-5;
	v_N = x(4);
	v_E = x(5);
	v_D = x(6);
	L_b = x(7);
	h_b = x(9);
	
	F = zeros(3,3);
	F(1,1) = v_D/(R_N(L_b) + h_b);
	F(1,2) = -2*v_E*tan(L_b)/(R_E(L_b) + h_b) - 2*omega_ie*sin(L_b);
	F(1,3) = v_N/(R_N(L_b) + h_b);
	F(2,1) = v_E*tan(L_b)/(R_E(L_b) + h_b) + 2*omega_ie*sin(L_b);
	F(2,2) = (v_N*tan(L_b) + v_D)/(R_E(L_b) + h_b);
	F(2,3) = v_E/(R_E(L_b) + h_b) + 2*omega_ie*cos(L_b);
	F(3,1) = -2*v_N/(R_N(L_b) + h_b);
	F(3,2) = -2*v_E/(R_E(L_b) + h_b) - 2*omega_ie*cos(L_b);
	F(3,3) = 0;
end

function F = F_23(x) 
	omega_ie = 7.292e-5;
	v_N = x(4);
	v_E = x(5);
	v_D = x(6);
	L_b = x(7);
	h_b = x(9);
	
	F = zeros(3,3);
	F(1,1) = -(v_E*sec(L_b))^2/(R_E(L_b) + h_b) - 2*v_E*omega_ie*cos(L_b);
	F(1,3) = v_E^2*tan(L_b)/(R_E(L_b) + h_b)^2 - v_N*v_D/(R_N(L_b) + h_b)^2;
	F(2,1) = v_N*v_E*sec(L_b)^2/(R_E(L_b) + h_b) - 2*v_N*omega_ie*cos(L_b) - 2*v_D*omega_ie*sin(L_b);
	F(2,3) = -(v_N*v_E*tan(L_b) + v_N*v_D)/(R_E(L_b) + h_b)^2;
	F(3,1) = 2*v_E*omega_ie*sin(L_b);
	F(3,3) = v_E^2/(R_E(L_b) + h_b)^2 + v_N^2/(R_N(L_b) + h_b)^2 - 2*g_0(L_b)/r_S(L_b);
end

function F = F_32(x)
	L_b = x(7);
	h_b = x(9);
	F = [
		1/(R_N(L_b) + h_b)	0	0;
		0	1/((R_E(L_b) + h_b)*cos(L_b))	0;
		0	0	-1];
end

function F = F_33(x) 
	v_N = x(4);
	v_E = x(5);
	L_b = x(7);
	h_b = x(9);
	F = zeros(3);
	F(1,3) = -v_N/(R_N(L_b) + h_b)^2;
	F(2,1) = v_E*sin(L_b)/((R_E(L_b) + h_b)^2*cos(L_b)^2);
	F(2,3) = -v_E/((R_E(L_b) + h_b)^2*cos(L_b));
end

function S = skew(x)
	S = [0		-x(3)	x(2);
		x(3)	0		-x(1);
		-x(2)	x(1)	0];
end

function R_e = R_E(L_b)
	R_0 = 6378137.0;
	R_p = 6356752.31425;
	e = sqrt(1 - R_p^2/R_0^2);
	R_e = R_0/sqrt(1-(e*sin(L_b))^2);
end

function R_n = R_N(L_b)
	R_0 = 6378137.0;
	R_p = 6356752.31425;
	e = sqrt(1 - R_p^2/R_0^2);
	R_n = R_0*(1-e^2)/(1-(e*sin(L_b))^2)^1.5;
end

function r = r_S(L_b)
	R_0 = 6378137.0;
	R_p = 6356752.31425;
	e = sqrt(1 - R_p^2/R_0^2);
	r = R_E(L_b)*sqrt(cos(L_b)^2+(1-e^2)^2*sin(L_b)^2);
end

function g = g_0(L_b)
	g = 9.80665;
end


