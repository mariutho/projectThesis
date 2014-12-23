function R = R_bn(Theta)
	phi = Theta(1);
	theta = Theta(2);
	psi = Theta(3);
	R_phi = [
		1	0			0;
		0	cos(phi)	-sin(phi);
		0	sin(phi)	cos(phi)];
	R_theta = [
		cos(theta)	0	sin(theta);
		0			1	0;
		-sin(theta)	0	cos(theta)];
	R_psi = [
		cos(psi)	-sin(psi)	0;
		sin(psi)	cos(psi)	0;
		0			0			1];
	
	R = R_psi*R_theta*R_phi;
end