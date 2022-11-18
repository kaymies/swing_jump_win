% This is the main MATLAB script for Lab 4.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
% const_point = [0; -0.15];
% const_point = [0.1; -0.13];
% const_point = [-0.16; -0.14];
const_point = [-0.16; -0.14]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
pts_foot = repmat(const_point,1,8);
pts_foot = [    0.0315    0.0315    0.0315    0.0911   -0.1857   -0.1857   -0.1857   -0.1857
   -0.2095   -0.2095   -0.2095   -0.0681   -0.0015   -0.1008   -0.1008   -0.1008];
       
%pts_foot = []; % YOUR BEZIER PTS HERE

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0; %shoulder
angle2_init = 0; %hip

% Total experiment time is buffer,trajectory,buffer
traj_time         = 6;
pre_buffer_time   = 0; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;

tis = 0.7; %shoulder start time
tih = 1.5; %hip start time

tipre = 0.1;
thpre = 0.5;%deg2rad(5); 
th2_limu = 1.0;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 60.0; %K1
gains.K_yy = 100.0; %K2
gains.K_xy = 0;

gains.D_xx = 0.01; %D1
gains.D_yy = 0.01; %D2
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max = 1.0;

%% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init, pts_foot,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       gains, duty_max, tis, tih, tipre, thpre, th2_limu);

%% Extract data
t = output_data(:,1);
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
   
xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydes = output_data(:,17); % desired foot position in Y

%% Plot foot vs desired
figure(3); clf;
subplot(211); hold on
plot(t,xdes,'r-'); plot(t,x);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,ydes,'r-'); plot(t,y);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});

figure(4); clf; hold on
plot(xdes,ydes,'r-'); plot(x,y,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});

% for Cartesian constant points, un-comment this to see the virtual potential well on figure 4
% [X, Y] = meshgrid(linspace(-0.25,0.25,50),linspace(-0.25,0.1,50));
% eX = X - (-const_point(1)); 
% eY = Y - const_point(2); 
% V = 0.5*gains.K_xx*eX.*eX + 0.5*gains.K_yy*eY.*eY + gains.K_xy*eX.*eY;
% axis([-0.25, 0.25, -0.25, 0.1]);
% contour(X,Y,V,15,'LineWidth',1.5);

