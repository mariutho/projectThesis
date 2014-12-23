function [H, H_b] = H_lin(x, dx)
	assert(length(x) == 15)
	assert(length(x) == 15)
	Theta = x(1:3);
	v = x(4:6);
	dTheta = dx(1:3);
	dv = dx(4:6);
	
	R_bl = eulerRot(Theta);
	R_lb = R_bl.';
	%H_Theta = -skew(-2*R_lb*dv + R_lb*v);
	%H_v = -R_lb-2*skew(dTheta)*R_lb;
	
	H_Theta = -skew(R_lb*v);
	H_v = -R_lb;
	
	z_3 = zeros(3);
	H_b = [H_Theta H_v z_3 z_3 z_3];
	H = E_D()*H_b;
end

function E = E_D()
	beta = -60*pi/180;
	alpha = pi/4 + pi/2*(0:3)';
	E = [cos(beta)*cos(alpha) cos(beta)*sin(alpha) sin(beta)*ones(4,1)];
end