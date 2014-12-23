function h_out = h_x_LS(x, dx)
	x_full = [0;0; x(1); x(2:3); 0];
	dx_full = [0;0;0; dx(2:3); 0];
	h = h_LS_full(x_full, dx_full);
	h_out = h(1:2);
end

function h = h_LS_full(x, dx) 
	dv = dx(4:6);
	dTheta = dx(1:3);
	v = x(4:6);
	%h = -dv - skew(v - dv)*dTheta;
	h = -dv;
end