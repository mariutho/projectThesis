function h = h_D(x, dx)
	dv = dx(4:6);
	dTheta = dx(1:3);
	Theta = x(1:3);
	v = x(4:6);
	R_bl = eulerRot(Theta);
	R_lb = R_bl.';
	E = E_D();
	%h = E*(-R_lb*dv - 2*skew(dTheta)*R_lb*dv + skew(dTheta)*R_lb*v);
	h = E*(-R_lb*dv + skew(dTheta)*R_lb*v);
%	h = -E*R_lb*dv;
end