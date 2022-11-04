function derive_everything() 
name = 'swing_jump_win';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy tha dtha ddtha thh dthh ddthh ths dths ddths real
syms taua tauh taus Fy Fy2 real
syms l0 l1 l2 l3 l4 real % 0-ankle; 1-tibia; 2-femur; 3-torso; 4-arm
syms c0 c1 c2 c3 c4 real 
syms m0 m1 m2 m3 m4 m5 real % m5 hand mass
syms I0 I1 I2 I3 I4 g real
syms le1 le2 k ls0 real % le1-distance from ankle to spring attachment point along -er0hat;
                       % le2-distance from ankle to spring attachment point
                       % along er1hat
                       % ls-natural length of spring

% Group them for later use.
q   = [y; tha; thh; ths];      % generalized coordinates
dq  = [dy; dtha; dthh; dths];    % first time derivatives
ddq = [ddy; ddtha; ddthh; ddths];  % second time derivatives
u   = [taua; tauh; taus];          % control forces and moments
Fc   = [Fy; Fy2];           % constraint forces and moments
p   = [l0; l1; l2; l3; l4;
       c0; c1; c2; c3; c4; 
	   m0; m1; m2; m3; m4; m5;
       I0; I1; I2; I3; I4; g;
       le1; le2; k; ls0];  % parameters

%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

% Define other unit vectors for use in defining other vectors.
er0hat = cos(-tha)*ihat - sin(-tha)*jhat;
er1hat =  cos(thh)*ihat + sin(thh)*jhat;
er2hat = -cos(thh)*ihat + sin(thh)*jhat;
er4hat = sin(ths)*ihat - cos(ths)*jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
rt = y*jhat + l0*er0hat;
rc0 = y*jhat + c0*er0hat;
ra = y*jhat;
rc1 = ra + c1*er1hat;
rk = ra + l1*er1hat;
rc2 = rk + c2*er2hat;
rh = rk + l2*er2hat;
rc3 = rh + c3*jhat;
rs = rh + l3*jhat;
rc4 = rs + c4*er4hat;
rf = rs + l4*er4hat;
keypoints = [rt ra rk rh rs rf]; %t-toe; a-ankle; k-knee; h-hip; s-shoulder; f-finger

% Take time derivatives of vectors as required for kinetic energy terms.
drc0 = ddt(rc0);
drc1 = ddt(rc1);
drc2 = ddt(rc2);
drc3 = ddt(rc3);
drc4 = ddt(rc4);
drf   = ddt(rf);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body.
T0 = (1/2)*m0*dot(drc0, drc0) + (1/2)* I0 * dtha^2;
T1 = (1/2)*m1*dot(drc1, drc1) + (1/2)* I1 * dthh^2;
T2 = (1/2)*m2*dot(drc2, drc2) + (1/2)* I2 * dthh^2;
T3 = (1/2)*m3*dot(drc3, drc3);
T4 = (1/2)*m4*dot(drc4, drc4) + (1/2)* I4 * dths^2;
T5 = (1/2)*m5*dot(drf, drf);

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
gamma = pi - thh + tha;
ls_current = sqrt(le1^2 + le2^2 - 2*le1*le2*cos(gamma));
V0 = m0*g*dot(rc0, jhat) + 1/2*k*(ls0-ls_current)^2;
V1 = m1*g*dot(rc1, jhat);
V2 = m2*g*dot(rc2, jhat);
V3 = m3*g*dot(rc3, jhat);
V4 = m4*g*dot(rc4, jhat);
V5 = m5*g*dot(rf, jhat);

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
% Not sure...
QF = F2Q(Fy*jhat,rt) + F2Q(Fy2*jhat,ra);
Qtaua = 0;
Qtauh = M2Q(-tauh*khat, -dthh*khat);
Qtaus = M2Q(taus*khat, dths*khat);

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
T = T0 + T1 + T2 + T3 + T4 + T5;
V = V0 + V1 + V2 + V3 + V4 + V5;
Q = QF + Qtaua + Qtauh + Qtaus;

% Calculate rcm, the location of the center of mass
rcm = (m0*rc0 + m1*rc1 + m2*rc2 + m3*rc3 + m4*rc4 + m5*rf)/(m0+m1+m2+m3+m4+m5);

% Assemble C, the set of constraints
C = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC= ddt(C);

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;


%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:4,1) = q;  
z(5:8,1) = dq;

% Write functions to a separate folder because we don't usually have to see them
directory = '../AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});