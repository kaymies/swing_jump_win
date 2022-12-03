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

    p = parameters();                           % get parameters from file

%     p(25) = input.k_curr;

%     input.AnimOn = 1;
%     input.PlotOn = 1;
%     tis = 1;

    %Tip foot
    z0 = [0.123; pi/2; 0.6454; -pi/2;...
          0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]

    %Foot flat
%     z0 = [0.2; pi/2; 0.64546; -pi/2;...
%           0; 0; 0; 0];                    % set initial state [y, tha, thh, ths]


    dths = input.dths; %shoulder angular velocity
    thsi = -pi/2; %Initial shoulder angle
    thsf = 3*pi/4; %Final shoulder angle

    % set guess
    tf = 1;                       % simulation final time
    ctrl.tih = 0.5;
    ctrl.tfh = 3;                 % control time points for hip - updated KS
    ctrl.Th = [0.1 0.1];          % control values for hip - updated KS
%     ctrl.Th = [0 0];
    ctrl.tis = tis;             %Start time for shoulder
    ctrl.thsi = thsi;            % Initial shoulder position
    ctrl.thsf = thsf;           % Final shoulder position
    ctrl.Ts = [0.1 0.1];          % control values for shoulder - updated KS
    % ctrl.Ts = [0 0];

    %Trajectory functions for arm swing
%     ramp = @(t) t .* heaviside(t);
%     tfs = tis + (thsf - thsi)/dths;
%     ctrl.thsfun = @(t) dths * (ramp(t - tis) - ramp(t - tfs)) + thsi;
%     ctrl.dthsfun = @(t) dths * (heaviside(t - tis) - heaviside(t - tfs));



    

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


        figure(7)
        plot(t,z(8,:) .* u(3,:))
        title("arm motor powervs time")
        hold on

        figure(6)
        plot(t, energy_swing_jump_win(z,p))
        xlabel('time (s)')
        ylabel('Energy')
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


