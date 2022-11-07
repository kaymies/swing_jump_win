function simulate_leg_template()
    %% Definte fixed paramters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    
    l0 =
    l1 =
    l2 =
    l3 =
    l4 =
    c0 =
    c1 =
    c2 =
    c3 =
    c4 =
    m0 =
    m1 =
    m2 =
    m3 =
    m4 =
    m5 =
    I0 =
    I1 =
    I2 =
    I3 =
    I4 =
    g =
    le1b =
    le2 =
    k =
    ls0 =
    
    restitution_coeff = 0.;
    friction_coeff = 0.3;
%     friction_coeff = 10;
    ground_height = -0.13;
    %% Parameter vector
%     p   = [m1 m2 m3 m4 I1 I2 I3 I4 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
    p = [t y dy ddy tha dtha ddtha thh dthh ddthh ths dths ddths taua tauh taus Fy Fy2 l0 l1 l2 l3 l4 c0 c1 c2 c3 c4 m0 m1 m2 m3 m4 m5 I0 I1 I2 I3 I4 g le1 le2 k ls0]';
    multiplier = 1;
%     p_fake   = [m1 m2 m3 m4 I1*multiplier I2 I3*multiplier I4 Ir*multiplier N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g m0 m1 m2 m3 m4 m5 I0 I1 I2 I3 I4 g le1 le2 k ls0]';
     
%     p_fake = give incorrect inertia and see how controller reacts, and
%     give to controller
    %% Simulation Parameters Set 2 -- Operational Space Control
    p_traj.omega = 30;
%     p_traj.omega = 3;
    p_traj.x_0   = 0;
    p_traj.y_0   = -.125;
    p_traj.r     = 0.025;
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 5;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    z0 = [-pi/4; pi/2; 0; 0];
    z_out = zeros(4,num_step);
    z_out(:,1) = z0;
    
    for i=1:num_step-1
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj);
        
%         qdot = discrete_impact_contact(z_out(:,i),p, restitution_coeff, friction_coeff, ground_height);
       
%         z_out(3:4,i) = qdot;
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
%         qdot = discrete_impact_contact(z_out(:,i+1),p, restitution_coeff, friction_coeff, ground_height);
%         z_out(3:4,i+1) = qdot;
%         qdot = joint_limit_constraint(z_out(:,i+1),p);
%         z_out(3:4,i+1) = qdot;
        % Position update
        z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt;
    end
    
    %% Compute Energy
    E = energy_swing_jump_win(z_out,p);
    figure(1); clf
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    %% Compute foot position over time
    rE = zeros(2,length(tspan));
    vE = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rE(:,i) = position_foot(z_out(:,i),p);
        vE(:,i) = velocity_foot(z_out(:,i),p);
    end
    
    figure(2); clf;
    plot(tspan,rE(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,p_traj.x_0 + p_traj.r * cos(p_traj.omega*tspan) ,'r--');
    plot(tspan,rE(2,:),'b','LineWidth',2)
    plot(tspan,p_traj.y_0 + p_traj.r * sin(p_traj.omega*tspan) ,'b--');
    
    
    xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','x_d','y','y_d'});

    figure(3); clf;
    plot(tspan,vE(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,vE(2,:),'b','LineWidth',2)
    
    xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
    
    figure(4)
    plot(tspan,z_out(1:2,:)*180/pi)
    legend('q1','q2');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    
    figure(5)
    plot(tspan,z_out(3:4,:)*180/pi)
    legend('q1dot','q2dot');
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/sec)');
    
    %% Animate Solution
    figure(6); clf;
    hold on
   
    % Target traj
    TH = 0:.1:2*pi;
    plot( p_traj.x_0 + p_traj.r * cos(TH), ...
          p_traj.y_0 + p_traj.r * sin(TH),'k--'); 
    
    % Ground Q2.3
    plot([-.2 .2],[ground_height ground_height],'k'); 
    
    animateSol(tspan, z_out,p);
end

function tau = control_law(t, z, p, p_traj)
    % Controller gains, Update as necessary for Problem 1
    K_x = 150.; % Spring stiffness X
    K_y = 150.; % Spring stiffness Y
    D_x = 10.;  % Damping X
    D_y = 10.;  % Damping Y

    % Desired position of foot is a circle
    omega_swing = p_traj.omega;
    rEd = [p_traj.x_0 p_traj.y_0 0]' + ...
            p_traj.r*[cos(omega_swing*t) sin(omega_swing*t) 0]';
    % Compute desired velocity of foot
    vEd = p_traj.r*[-sin(omega_swing*t)*omega_swing    ...
                     cos(omega_swing*t)*omega_swing   0]';
    % Desired acceleration
    aEd = p_traj.r*[-cos(omega_swing*t)*omega_swing^2 ...
                    -sin(omega_swing*t)*omega_swing^2 0]';
    
    % Actual position and velocity 
    rE = position_foot(z,p);
    vE = velocity_foot(z,p);
    
    % Compute virtual foce for Question 1.4 and 1.5
    f  = [K_x * (rEd(1) - rE(1) ) + D_x * ( - vE(1) ) ;
          K_y * (rEd(2) - rE(2) ) + D_y * ( - vE(2) ) ];
    
    %% Task-space compensation and feed forward for Question 1.8

    % Map to joint torques  
    J  = jacobian_foot(z,p);
    J_dot = jacobian_dot_foot(z,p);
    M = Mass_swing_jump_win(z, p);
    V = Corr_swing_jump_win(z, p);
    G = Grav_swing_jump_win(z, p);
    Alpha = inv(J*inv(M)*J');
    mu = Alpha*J*inv(M)*V-Alpha*J_dot*z(3:4);
    rho = Alpha*J*inv(M)*G;
%     tau = J' * f;
    K = [K_x; K_y];
    D = [D_x; D_y];
%     temp = K*(rEd-rE)'
%     temp2 = K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)
    tau = J'*(Alpha*(aEd(1:2) + K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)) + mu + rho);
%     tau = J'*(Alpha*(aEd(1:2) + K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)) + mu + rho+0.2);
%     tau = J'*(Alpha*(aEd(1:2) + K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)) + mu);
%     tau = J'*(Alpha*(aEd(1:2) + K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)) + rho);
%     tau = J'*(Alpha*(K.*(rEd(1:2)-rE) + D.*(vEd(1:2)-vE)) + mu + rho);
end


function dz = dynamics(t,z,p,p_traj)
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    % Get mass matrix
    A = A_swing_jump_win(z,p);
    
    % Compute Controls
    multiplier = 2;
    p_fake   = [m1 m2 m3 m4 I1*multiplier I2*multiplier I3*multiplier I4*multiplier Ir*multiplier N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
    tau = control_law(t,z,p_fake,p_traj);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_swing_jump_win(z,tau,p);
    
%     qdot = discrete_impact_contact(z,p, restitution_coeff, friction_coeff, ground_height);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
%     dz(1:2) = qdot;
    dz(3:4) = qdd;
end

function qdot = discrete_impact_contact(z,p, rest_coeff, fric_coeff, yC)
    r_E = position_foot(z,p);
    Cy = r_E(2)-yC;
    vE = velocity_foot(z,p);
    Cdot = vE(2);
    
    J = jacobian_foot(z,p);

    M = Mass_swing_jump_win(z, p);
    Alpha = inv(J*inv(M)*J');
%     if z(2) < 0
%         print "now"
%     end
%     if Cy <= 0
%        print "here" 
%     end
%     if Cy <= 0 && Cdot <= 0
%         J_x = J(1,:);
%         J_y = J(2,:);
% %         Alpha_x = Alpha(1, 1); % ------------- CHECK --------------
% %         Alpha_y = Alpha(2, 2);
%         Alpha_x = inv(J_x*inv(M)*J_x');
%         Alpha_y = inv(J_y*inv(M)*J_y');
%         Fc_y = Alpha_y*(-rest_coeff*Cdot- J_y*z(3:4));
%         qdot = z(3:4) + inv(M)*J_y'*Fc_y;
%         
%         Fc_x = Alpha_x*(0 - J_x*z(3:4));
%         if Fc_x > fric_coeff*Fc_y
%             Fc_x = fric_coeff*Fc_y;
%         elseif Fc_x < -fric_coeff*Fc_y
%             Fc_x = -fric_coeff*Fc_y;
%         end
%         qdot = qdot + inv(M)*J_x'*Fc_x;
%     else
%         qdot = z(3:4);
%     end
    
    
end

function qdot = joint_limit_constraint(z,p)
   
    q2_d = deg2rad(-50);
    J = jacobian_foot(z,p);
    J_dot = jacobian_dot_foot(z,p);
    M = Mass_swing_jump_win(z, p);
    V = Corr_swing_jump_win(z, p);
    G = Grav_swing_jump_win(z, p);
    C = z(1) - q2_d;
%     Tauc = -Kappa_c*C-Dampa_c*z(4);
    
%     if C < 0 && z(3)<0
%         
% %         Alpha = inv(J*inv(M)*J');
% % 
% %         Fc = Alpha*(0 - J*z(3:4));
%         
%         Tau_c = M*(0 - z(3:4));
% 
% %         if Fc(2) > fric_coeff*Fc_y
% %             Fc_x = fric_coeff*Fc_y;
% %         elseif Fc(2) < -fric_coeff*Fc_y
% %             Fc_x = -fric_coeff*Fc_y;
% %         end
% %         qdot = z(3:4) + inv(M)*J'*Fc;
%         qdot = z(3:4) + inv(M)*Tau_c;
%     else
%         qdot = z(3:4);
%     end
    
end

function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
   
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.3 .1]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_swing_jump_win(z,p);

        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rD = keypoints(:,4);
        rE = keypoints(:,5);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OB,'XData',[0 rB(1)]);
        set(h_OB,'YData',[0 rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);

        pause(.01)
    end
end