close all; clear all; clc;

input.AnimOn = 1;
input.PlotOn = 0;
input.k_curr = 0;
input.dths = 0;

k_curr = 4*70;
input.k_curr = k_curr;


% input.dths = dths;
% t_swing = (pi/2 + 3/4*pi)/dths; %Theoretical arm swing time
% baseline_peak = run_simulation(1,input); %Peak with no arm swing
% ti = 0.5 - t_swing - 0.1; %Time range based on how long it takes for the arm to swing
% tf = 0.5 + 0.2;     %Time base on how long it takes to reach jump peak without arm swing

ti = 0.5 - 0.12;
tf = 0.5 + 0.17;

tis = ti:0.001:tf;
tis = 0.5 -0.045; %-0.073, -0.07, - 0.045, -0.018, -0.018, 0.015, 0.039
% tis = tis(10);
peaks = zeros(1,length(tis));
for i = 1:length(tis)
    out = run_simulation(tis(i),input);
    peaks(i) = out;
    i
end

%%
input.AnimOn = 0;
input.PlotOn = 0;
baseline_peak = run_simulation(1,input); %Peak with no arm swing
% baseline_peak = 0;
figure();
plot(tis-0.5,peaks-baseline_peak)
title(strcat("Parameter Sweep for k=",num2str(k_curr)," N-m, and arm speed = ",num2str(dths)," rad/s"))
xlabel('Time Difference (Shoulder-Hip) [s]')
ylabel('Peak Height, Relative To No Swing [m]')
