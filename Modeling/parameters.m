function p = parameters() 
 l0 = 51.52*1e-3;
 l1 = 91*1e-3;
 l2 = 64*1e-3;
 l3 = 55.84*1e-3;
 l4 = 54.8*1e-3;
 c0 = 0;
 c1 = 53.43*1e-3;
 c2 = 34.09*1e-3;
 c3 = 34.5*1e-3;
 c4 = 19.39*1e-3;
 m0 = 1.67*1e-3;
 m1 = 18.33*1e-3;
 m2 = 990*1e-3;
 m3 = 34.9*1e-3;
 m4 = 8.22*1e-3;
 m5 = 100*1e-3;
 I0 = 490.45;
 I1 = 10865.5;
 I2 = 4668.15;
 I3 = 39244;
 I4 = 3428.51;
 le1 = l0/2;
 le2 = 41.63*1e-3;
 k = 30; %% TEMPORARY - BALLPARK STIFFNESS FOR RUBBER BAND
 ls0 = 38E-3; %% TEPORARY
 g = 9.81;
 p   = [l0; l1; l2; l3; l4;
       c0; c1; c2; c3; c4; 
	   m0; m1; m2; m3; m4; m5;
       I0; I1; I2; I3; I4; g;
       le1; le2; k; ls0];
end