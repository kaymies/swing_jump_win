function [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,tspan,t_start)
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
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(3,1);
    iphase_list = 1;
    for i = 1:num_step-1
        t = tout(i);
        %%% SG Edits
        iphase = iphase_list(i);
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase,t_start);
        zout(:,i+1) = zout(:,i) + dz*dt;
        zout(5:8,i+1) = discrete_impact_contact(zout(:,i+1), p);
        zout(1:4,i+1) = zout(1:4,i) + zout(5:8, i+1)*dt;
        uout(:,i+1) = u; 
        %%%% END
        if(zout(1,i+1) > 0 && iphase == 1) % jump
            iphase = 2;
        elseif(zout(1,i+1) < 0 && iphase == 2) % max height
            iphase = 3;
        end
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
%     qdot = z(5:8);
%     rE = z(1); 
%     vE = z(5); 
% 
%     if(rE<0 && vE < 0)
%       J  = [1, 0];
%       M = A_swing_jump_win(z,p);
%       Ainv = inv(M);
%       
%       lambda_z = 1/(J * Ainv * J.');
%       F_z = lambda_z*(0 - vE);
%       qdot = qdot + Ainv*J.'*F_z;
    qdot = z(5:8);
    rt = r_toe_swing_jump_win(z,p);
    rty = rt(2);
    vt = v_toe_swing_jump_win(z,p);
    vty = vt(2);
    
    if(rty < 0 && vty < 0)
      
      M = A_swing_jump_win(z,p);
      Ainv = inv(M);

      Jt  = J_toe_swing_jump_win(z,p);
      Jty = Jt(2,:);
      
      lambda_z = 1/(Jty * Ainv * (Jty.'));
      F_y = lambda_z*(0 - vty);
      qdot = qdot + Ainv*Jty.'*F_y;
    end
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase,t_start)
    %UPDATED - SG
    % 03 Nov 2022 - updated dz to be 8x1 vector instead of 4x1 - SG
    u = control_laws(t,z,ctrl,iphase,t_start);  % get controls at this instant
    
    A = A_swing_jump_win(z,p);                 % get full A matrix
    b = b_swing_jump_win(z,u,[0;0],p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8);     % assign velocities to time derivative of state vector
    dz(5:8,1) = x(1:4);     % assign accelerations to time derivative of state vector
end

%% Control
function u = control_laws(t,z,ctrl,iphase,t_start)
    %UPDATED - SG
    %03 Nov 2022 - adjusted areal control so that PD is applied to leg, but
    %arm control is preserved. Adjusted in-ground control to output leg and
    %arm control curves.
%     if iphase == 1
%         %Ankle control, irrelevant
%         taua = 0;
%         %Leg control, Bezier curve on torque
%         tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
%         %Arm control
%         taus = BezierCurve(ctrl.Ts, t/ctrl.tfs); %EDIT LATER TO MATCH CONTROL LAW
% 
%         %Create control vector
%         u = [taua; tauh; taus];
%     else
%         %Updated - th and dth as vector of ankle, hip, shoulder angles - KS
%         % PD Control in flight
%         th = z(2:4,:);            % leg angle
%         dth = z(6:8,:);           % leg angular velocity
% 
%         thd = pi/4;             % desired leg angle
%         k = 5;                  % stiffness (N/rad)
%         b = 0.5;                 % damping (N/(rad/s))
% 
%         u = -k*(th-thd) - b*dth;% apply PD control
% %         u(1) = -k*(th(1)+thd) - b*dth(1); %Ankle want negative target angle
%         u(1) = 0;
%     end
%____________________________________________________________________________
%     %UPDATED - EK 
%     %07 Nov 2022 - restrict max torque in leg and arm according to min
%     %velocity tested. 
%     inv_Kt = 0.0132; 
%     min_vel = 2.75;
%     max_torque = -inv_Kt*min_vel + 0.85;
%     if iphase == 1
%         %Ankle control, irrelevant
%         taua = 0;
% 
%         % motor equation torque = -0.0132*speed + 0.85
%         %UPDATED - EK 
%         %07 Nov 2022 - max torque in leg and arm
%         
% %         speed_min = 2.75;
%         tauh = -inv_Kt*z(7) + 0.85;
% %         tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
%         %Arm control
%         taus = -inv_Kt*z(8) + 0.85;
% %         taus = BezierCurve(ctrl.Ts, t/ctrl.tfs); %EDIT LATER TO MATCH CONTROL LAW
%         if tauh > max_torque
%             tauh = max_torque;
%         end
%         if taua > max_torque
%             taua = max_torque;
%         end
%         %Create control vector
%         u = [taua; tauh; taus];
%     else
%         %Updated - th and dth as vector of ankle, hip, shoulder angles - KS
%         % PD Control in flight
%         th = z(2:4,:);            % leg angle
%         dth = z(6:8,:);           % leg angular velocity
% 
%         th_des = [-pi/6; pi/4; pi/2]; % shoulder straight up
%         dth_des = [0; min_vel; min_vel];
%         
%         th_des = [0.0; 0.0; 0.0];             % desired leg angle
%         dth_des = [0.0; 0 ;0];
%         
%         k = 5;                  % stiffness (N/rad)
%         b = .5;                 % damping (N/(rad/s))
% 
%         u = -k*(th-th_des) - b*(dth-dth_des);% apply PD control
%         u(1,:) = 0; % Ankle torque is zero
%         if u(2,:) > max_torque
%             u(2,:) = max_torque;
%         end
%         if u(3,:) > max_torque
%             u(3,:) = max_torque;
%         end
%     end
%     if t < t_start(1) % Check if haven't reached hip torque trajectory start time
%         u(2,:) = 0;
%     end
%     if t < t_start(2) % Check if haven't reached shoulder torque trajectory start time
%         u(3,:) = 0;
%     end
%     u(3,:) = 0;
%     taua = 0;
%     tauh = 0;
%     taus = 0;

%____________________________________________________________________________
    % UPDATED EK
    %11 Nov 2022 - max torque in leg and arm
    % motor equation torque = -0.0132*speed + 0.85 from curve provided in
    % SLACK
    inv_Kt = 0.0132; 
    min_vel = 2.75;
    max_torque = -inv_Kt*min_vel + 0.85;
    tauh = -inv_Kt*z(7) + 0.85;
%         tauh = BezierCurve(ctrl.Th, t/ctrl.tfh); %EDIT LATER TO MATCH CONTROL LAW
    %Arm control
    taus = -inv_Kt*z(8) + 0.85;
%         taus = BezierCurve(ctrl.Ts, t/ctrl.tfs); %EDIT LATER TO MATCH CONTROL LAW
    if tauh > max_torque % check on max torque to be within motor limits
        tauh = max_torque;
    end
    if taus > max_torque
        taus = max_torque;
    end
   

    if t < t_start(1) % Check if haven't reached hip torque trajectory start time
        tauh = 0;
    end
    if t < t_start(2) % Check if haven't reached shoulder torque trajectory start time
        taus = 0;
    end
    
    taua = 0; % Ankle always zero torque
%     tauh = 0; % ZERO TORQUE CASE
    taus = 0; % Fix shoulder for now
    
    %Create control vector
    u = [taua; -tauh; taus]; % hip torque should be negative to jump up

end
