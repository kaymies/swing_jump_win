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
z0 = [0.035; -pi/4; pi/4; pi/2;...
      0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]

% z0 = [0.1; -pi/4; pi/4; pi/2;...
%       0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]

%         tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
%         %Arm control
%         taus = BezierCurve(ctrl.Ts, t/ctrl.tfs);

% set guess
tf = 1;                                        % simulation final time
ctrl.tfh = 1;                                  % control time points for hip - updated KS
ctrl.Th = [1 1];                               % control values for hip - updated KS
% ctrl.Th = [0 0];
ctrl.tfs = 1;                                  % control time points for shoulder - updated KS
ctrl.Ts = [1 1];                               % control values for shoulder - updated KS
ctrl.Ts = [0 0];

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

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

%% Plot COM for your submissions
figure(1)
COM = COM_swing_jump_win(z,p);
max(COM(2,:))
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

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
speed = 0.25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation