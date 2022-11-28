function p = parameters() 
 l0 = 51.52*1e-3;
 l1 = 91*1e-3;
 l2 = 64*1e-3;
 l2 = l1;
 l3 = 55.84*1e-3;
 l4 = 54.8*1e-3;
 c0 = 0;
 c1 = l1 - 53.43*1e-3;
 c2 = l2 - 34.09*1e-3;
 c3 = 34.5*1e-3;
 c4 = 19.39*1e-3;
 m0 = 1.852*1e-3;
 m1 = 18.64*1e-3;
 m2 = 11.86*1e-3;
 m3 = (434+100)*1e-3 - 0.453592;
 m4 = 8.22*1e-3;
 m5 = 75*1e-3;
 I0 = 490.45e-9;
 I1 = 10440e-9;
 I2 = 4668.15e-9;
 I3 = 39244e-9;
 I4 = 3428.51e-9;
 le1 = l0/2;
 le2 = 41.63*1e-3;
 k = 12*70; %% TEMPORARY - BALLPARK STIFFNESS FOR RUBBER BAND
%  k = 0;
%  ls0 = sqrt(le1^2 + le2^2); %% TEPORARY
 ls0 = 45.72*(1e-3)/2;
 g = 9.81;
%  g = 0;
 p   = [l0; l1; l2; l3; l4;
       c0; c1; c2; c3; c4; 
	   m0; m1; m2; m3; m4; m5;
       I0; I1; I2; I3; I4; g;
       le1; le2; k; ls0];
end