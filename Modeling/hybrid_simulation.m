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

    % Toe contact - SG
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
      qchange = Ainv*Jy.'*F_y;
      qdot = qdot + qchange;
    end

        % SG - Add joint position constraints - 07 Nov 2022
    
%     Ankle joint limit angle
    tha_lim0 = deg2rad(30); %Minimum angle
    tha_lim1 = deg2rad(95); %Maximum
    k = 500;
    c = 0.1;
    if z(2) < tha_lim0 && qdot(2) < 0
%         qdot(2) = qdot(2) + (k * (tha_lim0 - z(2)) - c * z(6));
%         qdot(2) = -1*qdot(2);
        z(2) = tha_lim0 + 0.01;
        qdot(2) = 0;
    elseif z(2) > tha_lim1 && qdot(2) > 0
        qdot(2) = 0;
        z(2) = tha_lim1 - 0.01;
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
      qchange = Ainv*Jy.'*F_y;
      qdot = qdot + qchange;
    end



end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)
    %UPDATED - SG
    % 03 Nov 2022 - updated dz to be 8x1 vector instead of 4x1 - SG
    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
%     u = [0;0;0];
    u2 = control_contacts(t,z);
%     u2 = [0;0;0];
    u = u + u2;
%     u = [0;0;0];
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

%         rpm_nl = 530; %RPM from Pololu data sheet
%         tau_stall = 0.834; %N-m from Pololu data sheet
%         omega_nl = rpm_nl*2*pi/60;
%         inv_Kt = tau_stall/omega_nl;

        tau_stall = 0.85; 
        inv_Kt = 0.0132;
        scalar = 1; %0-1 magnitude scalar to scale down torques
        
        %Ankle control, irrelevant
        taua = 0;
        tauh = 0;
        taus = 0;
        
        %Leg control
        if t >= ctrl.tih
            %Apply maximum effort
            tauh = scalar*(tau_stall - inv_Kt*z(7));
        end


        %Arm control
        Ks = 2;
        bs = 0.05;

        % PD control to match specified functions for th and dth
%         fun_th = ctrl.thsfun;
%         fun_dth = ctrl.dthsfun;
% 
%         ths_target = fun_th(t);
%         dths_target = fun_dth(t);

%         taus = Ks*(ths_target - z(4)) + bs*(dths_target-z(8));
%         taus_lim = scalar * (max(tau_stall - inv_Kt * abs(z(8)),0));

%         if abs(taus) > taus_lim
%             %cap off taus at limit;
%             if taus < 0
%                 taus = -1*taus_lim;
%             else
%                 taus = taus_lim;
%             end
%         end


        if t >= ctrl.tis 
            %Run shoulder with PD matching to final position
            taus = Ks*(ctrl.thsf - z(4)) + bs*(0-z(8));

            taus_lim = scalar * (max(tau_stall - inv_Kt * abs(z(8)),0));

            if abs(taus) > taus_lim
                %cap off taus at limit;
                if taus < 0
                    taus = -1*taus_lim;
                else
                    taus = taus_lim;
                end
            end
        else
            % Hold shoulder at init postn.
            taus = Ks*(ctrl.thsi - z(4))+ bs*(0-z(8)); 
        end


        %Create control vector
        u = [taua; tauh; taus];

end


% SG - Separate function for joint limits implemented as spring system - 13 Nov 2022

function u = control_contacts(t,z)
    taua = 0;
    taus = 0;
    tauh = 0;

    %SG - hip joint spring limiter instead of hard contact - 13 Nov 2022
    thh_lim0 = deg2rad(180-143); %Minimum angle
%     thh_lim0 = deg2rad(30); %Minimum angle
    thh_lim1 = deg2rad(180-106); %Maximum angle
    kH = 100;
    cH = 0.1;
    if z(3) > thh_lim1
        tauh = kH * (thh_lim1 - z(3)) - cH * z(7);
    elseif z(3) <= thh_lim0
        tauh = kH * (thh_lim0 - z(3)) - cH * z(7);
    end

    %SG - Ankle joint spring limiter instead of hard contact - 29 Nov 2022
%     tha_lim0 = deg2rad(30); %Minimum angle
%     tha_lim1 = deg2rad(95); %Maximum
%     kA = 1;
%     cA = 0.0001;
%     if z(2) > tha_lim1
%         taua = kA * (tha_lim1 - z(2)) - cA * z(6);
%     elseif z(2) <= tha_lim0
%         taua = kA * (tha_lim0 - z(2)) - cA * z(6);
%     end

    u = [taua; tauh; taus];
end





%%%% Scrap code

                        %Get torque and rotation directions
%             abs_rot_dir = sign(z(8)); %current rotation direction
%             if abs_rot_dir == 0
%                 abs_rot_dir = 1;
%             end
%             rel_torque_dir = sign(taus) * abs_rot_dir; %current torque direction relative to rotation - positive = same, negative = opposite, zero = irrelevant
%             if rel_torque_dir == 0
%                 rel_torque_dir = 1;
%             end

            % Calc motor torque limit. If going againts omega, than omega
            % is "negative" and taus_lim will be greater than tau_stall. If
            % going with omega and omega > omega_nl, then tau_lim is
            % negative (i.e., the motor will actually resist forward
            % motion).
%             taus_lim = tau_stall - inv_Kt * rel_torque_dir * abs(z(8)); 
