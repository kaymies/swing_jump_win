function [peak] = run_simulation(tis,input)
%     clear all; close all; clc;

    % We can organize our code by filing things in different folders.  These
    % folders need to be added to the Matlab path so that it can run the files
    % inside them even when they are not the current folder listed at the top
    % of the Matlab window.  For more information about the current folder, see
    % http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
    % For more information about the Matlab path, see
    % http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
    setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

    N = 100; % number of control intervals

    opti = casadi.Opti(); % Optimization problem
    
    p = parameters();                           % get parameters from file

%     input.AnimOn = 1;
%     input.PlotOn = 1;
%     tis = 1;

    % Fix torque input, hip start time, start and end angle, total simulation time
    % Find state trajecotry, activation time of shoulder, and final control time
      
    z = opti.variable(8, N+1);  
    ctrl.tf = opti.variable();
    ctrl.tis = opti.variable();
    
    ctrl.tf = SX.sym('tf');
    ctrl.tih = SX.sym('tih');
    ctrl.tfh = SX.sym('tfh');
    z = SX.sym('z');
    
    %Tip foot
    z0 = [0.15; pi/2; 0.6454; -pi/2;...
          0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]
      
 
    % set guess
    tf = 1;                                        % simulation final time
    ctrl.tih = 0.5;
    ctrl.tfh = 3;                                  % control time points for hip - updated KS
    ctrl.Th = [0.1 0.1];                               % control values for hip - updated KS
%     ctrl.Th = [0 0];
    ctrl.tis = tis;
    ctrl.tfs = 1;                                  % control time points for shoulder - updated KS
    ctrl.thsi = -pi/2;
    ctrl.thsf = 3*pi/4;
    ctrl.Ts = [0.1 0.1];                               % control values for shoulder - updated KS
    % ctrl.Ts = [0 0];
    
    
    
    opti.set_initial(speed, 1);
    opti.set_initial(T, 1);

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
    
    % ---- solve NLP              ------
    opti.solver('ipopt'); % set numerical backend
    sol = opti.solve();   % actual solve


    %Extract solved parameters
    % tf = x(1);
    % ctrl.tf = x(2);
    % ctrl.T = x(3:end);

    [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation
    COM = COM_swing_jump_win(z,p);
    [peak, t_peak] = find_first_peak(t,z(1,:),COM,ctrl.tih,ctrl.tis);
    %% Plot COM for your submissions
    if input.PlotOn
        figure(1)
        max(COM(2,:))
        plot(t,COM(2,:))
        xlabel('time (s)')
        ylabel('CoM Height (m)')
        title('Center of Mass Trajectory')

        figure(2); clf
        yyaxis left
        plot(t,z(3,:))
        hold on
        yyaxis right
        plot(t,mod(z(4,:),2*pi))
        title("Hip v arm angle")
        legend("hip","arm")

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


        figure(6)
        plot(t, energy_swing_jump_win(z,p))
        xlabel('time (s)')
        ylabel('Energy')

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
    end
    %%
    % Run the animation
    if input.AnimOn
        figure(3)                          % get the coordinates of the points to animate
        speed = 0.1;                                 % set animation speed
        clf                                         % clear fig
        animate_simple(t,z,p,speed)                 % run animation
    end
end