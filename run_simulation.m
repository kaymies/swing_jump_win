clear all; close all; clc;

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
<<<<<<< Updated upstream
z0 = [0.4; -pi/6; pi/6; pi/6; 0 ; 0; 0 ;0];                    % set initial state
% Note: 5th state is the integral of torque squared over time
% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.
=======

% UPDATED EK
%11 Nov 2022 - starting y height should have toe on ground (not ankle),
%min_speed corresponds to max torque according to experimental results in
%SLACK. Added variable t_start to control when hip and shoulder torque
%start acting
min_speed = 2.75;
% z0 = [0.0; -pi/6; pi/6; pi/2; 0.0 ; min_speed; min_speed ;0.0];

t_start = [1.0, 0.0]; % Hip and shoulder torque start time
ankle_angle = -pi/6;
foot_len = p(1)-p(23);
Ankle_start_h = -1*foot_len*sin(ankle_angle);
z0 = [Ankle_start_h; ankle_angle; pi/6; 0;...
      0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]

% z0 = [0.1; -pi/4; pi/4; pi/2;...
%       0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]
>>>>>>> Stashed changes

%         tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
%         %Arm control
%         taus = BezierCurve(ctrl.Ts, t/ctrl.tfs);

% set guess
<<<<<<< Updated upstream
tf = 0.5;                                        % simulation final time
ctrl.tfh = 0.5;                                  % control time points for hip - updated KS
ctrl.Th = [0.5 0.5 0.5];                               % control values for hip - updated KS
ctrl.tfs = 0.5;                                  % control time points for shoulder - updated KS
ctrl.Ts = [0.5 0.5 0.5];                               % control values for shoulder - updated KS
=======
tf = 1.1;                                        % simulation final time
ctrl.tfh = 2;                                  % control time points for hip - updated KS
ctrl.Th = [0 0];                               % control values for hip - updated KS
ctrl.Th = [0 0];
ctrl.tfs = 1;                                  % control time points for shoulder - updated KS
ctrl.Ts = [0.0 0.0];                               % control values for shoulder - updated KS
ctrl.Ts = [0 0];
>>>>>>> Stashed changes

% x = [tf, ctrl.tf, ctrl.T];
% % setup and solve nonlinear programming problem
% problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
% problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
% problem.x0 = [tf ctrl.tf ctrl.T];                   % initial guess for decision variables
% problem.lb = [.4 .1 -2*ones(size(ctrl.T))];     % lower bound on decision variables
% problem.ub = [1  1   2*ones(size(ctrl.T))];     % upper bound on decision variables
% problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
% problem.Aeq = []; problem.beq = [];             % no linear equality constraints
% problem.options = optimset('Display','iter');   % set options
% problem.solver = 'fmincon';                     % required
% x = fmincon(problem);                           % solve nonlinear programming problem

% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

%Extract solved parameters
% tf = x(1);
% ctrl.tf = x(2);
% ctrl.T = x(3:end);

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf],t_start); % run simulation

%% Plot COM for your submissions
figure(1)
COM = COM_swing_jump_win(z,p);
max(COM(2,:))
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

<<<<<<< Updated upstream
=======
% figure(2)
% plot(t,z(2,:));

%UPDATED - EK 
%03 Nov 2022 - max torque in leg and arm
figure(4)
plot(t,u(1,:))
hold on
plot(t,u(2,:))
hold on
plot(t,u(3,:))
xlabel('time (s)')
ylabel('Torque (Nm)')
title('Torque Trajectory')
legend("Ankle torque", "Hip torque","Shoulder Torque")

figure(5)
plot(t,z(6,:))
hold on
plot(t,z(7,:))
hold on
plot(t,z(8,:))
xlabel('time (s)')
ylabel('Angular Velocity (rad/s)')
title('Velocity Trajectory')
legend("Ankle velocity", "Hip velocity","Shoulder velocity")


>>>>>>> Stashed changes
% figure(2)  % control input profile
% ctrl_t = linspace(0, ctrl.tf, 50);
% ctrl_pt_t = linspace(0, ctrl.tf, length(ctrl.T));
% n = length(ctrl_t);
% ctrl_input = zeros(1,n);
% 
% for i=1:n
%     ctrl_input(i) = BezierCurve(x(3:end),ctrl_t(i)/ctrl.tf);
% end
% 
% hold on
% plot(ctrl_t, ctrl_input);
% plot(ctrl_pt_t, x(3:end), 'o');
% hold off
% xlabel('time (s)')
% ylabel('torque (Nm)')
% title('Control Input Trajectory')
%%
% Run the animation
figure(3)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation