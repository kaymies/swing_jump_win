rot_z = [cos(pi/4) -sin(pi/4);
         sin(pi/4) cos(pi/4)];
K = [40 0;
     0 0];
K_rot = rot_z'*K*rot_z