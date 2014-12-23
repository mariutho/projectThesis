function E = E_D()
	beta = -60*pi/180;
	alpha = pi/4 + pi/2*(0:3)';
	E = [cos(beta)*cos(alpha) cos(beta)*sin(alpha) sin(beta)*ones(4,1)];
end