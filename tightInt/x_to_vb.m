function v_b = x_to_vb( x )
	v_l = x(2:3);
	yaw = x(1);
	R_lb = [cos(yaw) sin(yaw); -sin(yaw) cos(yaw)];
	v_b = R_lb*v_l;
end