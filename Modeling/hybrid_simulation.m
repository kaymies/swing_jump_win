function [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,tspan)
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
%
    t0 = tspan(1); tend = tspan(end);   % set initial and final times
    dt = 0.0001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(3,1);
    iphase_list = 1;
    for i = 1:num_step-1
        t = tout(i);
        %%% SG Edits
        iphase = iphase_list(i);
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase);
        zout(:,i+1) = zout(:,i) + dz*dt;
        zout(5:8,i+1) = discrete_impact_contact(zout(:,i+1), p);
        zout(1:4,i+1) = zout(1:4,i) + zout(5:8, i+1)*dt;
        uout(:,i+1) = u; 
        %%%% END
%         if(zout(1,i+1) > 0.0365 && iphase == 1) % jump
%             iphase = 2;
%         elseif(zout(1,i+1) < 0 && iphase == 2) % max height
%             iphase = 3;
%         end
        iphase_list(i+1) = iphase;
    end
    
    j=1;
    indices = 0;
    for i = 1:num_step-1
        if (iphase_list(i+1) == iphase_list(i))
            indices(j) = indices(j)+1;
        else
            j = j+1;
            indices(j) = 0;
        end
    end
end

%% Discrete Contact
function qdot = discrete_impact_contact(z,p)
    %UPDATED - SG
    % 06 Nov 2022 - z needs to match 8x1 format, for now just registering
    % contact of toe. Later, will register contact of ankle with ground as
    % well.
    qdot = z(5:8);
    rt = r_toe_swing_jump_win(z,p);
    rty = rt(2);
    vt = v_toe_swing_jump_win(z,p);
    vty = vt(2);
    
    if(rty < 0 && vty < 0)
      
      M = A_swing_jump_win(z,p);
      Ainv = inv(M);

      J  = J_toe_swing_jump_win(z,p);
      Jy = J(2,:);
      
      lambda_z = 1/(Jy * Ainv * (Jy.'));
      F_y = lambda_z*(0 - vty);
      qdot = qdot + Ainv*Jy.'*F_y;
    end

    % SG - Add ankle contact - 07 Nov 2022
    ra = r_ank_swing_jump_win(z,p);
    ray = ra(2);
    va = v_ank_swing_jump_win(z,p);
    vay = va(2);
    
    if(ray < 0 && vay < 0)
      
      M = A_swing_jump_win(z,p);
      Ainv = inv(M);

      J  = J_ank_swing_jump_win(z,p);
      Jy = J(2,:);
      
      lambda_z = 1/(Jy * Ainv * (Jy.'));
      F_y = lambda_z*(0 - vay);
      qdot = qdot + Ainv*Jy.'*F_y;
    end

    % SG - Add joint position constraints - 07 Nov 2022
    
    % Ankle joint limit angle
    tha_lim0 = deg2rad(-60); %Minimum angle
    tha_lim1 = deg2rad(0); %Maximum
    if z(2) < tha_lim0 && qdot(2) < 0
        qdot(2) = 0;
%     elseif z(2) > tha_lim1 && qdot(2) > 0
%         qdot(2) = 0;
    end

    % Hip joint limit angle
    thh_lim0 = deg2rad(180-143); %Minimum angle
    thh_lim1 = deg2rad(180-106); %Maximum angle
    if z(3) < thh_lim0 && qdot(3) < 0
        qdot(3) = 0;
    elseif z(3) > thh_lim1 && qdot(3) > 0
        qdot(3) = 0;
    end
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)
    %UPDATED - SG
    % 03 Nov 2022 - updated dz to be 8x1 vector instead of 4x1 - SG
    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
    
    A = A_swing_jump_win(z,p);                 % get full A matrix
    b = b_swing_jump_win(z,u,[0;0],p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8);     % assign velocities to time derivative of state vector
    dz(5:8,1) = x(1:4);     % assign accelerations to time derivative of state vector
end

%% Control
function u = control_laws(t,z,ctrl,iphase)
    %UPDATED - SG
    %03 Nov 2022 - adjusted areal control so that PD is applied to leg, but
    %arm control is preserved. Adjusted in-ground control to output leg and
    %arm control curves.
    if iphase == 1
        %Ankle control, irrelevant
        taua = 0;
        %Leg control, Bezier curve on torque
        tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
        %Arm control
        taus = BezierCurve(ctrl.Ts, t/ctrl.tfs); %EDIT LATER TO MATCH CONTROL LAW

        %Create control vector
        u = [taua; tauh; taus];
    else
        %Updated - th and dth as vector of ankle, hip, shoulder angles - KS
        % PD Control in flight
        th = z(2:4,:);            % leg angle
        dth = z(6:8,:);           % leg angular velocity

        thd = pi/4;             % desired leg angle
        k = 5;                  % stiffness (N/rad)
        b = 0.5;                 % damping (N/(rad/s))

        u = -k*(th-thd) - b*dth;% apply PD control
%         u(1) = -k*(th(1)+thd) - b*dth(1); %Ankle want negative target angle
        u(1) = 0;
    end

end