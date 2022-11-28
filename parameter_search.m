close all; clear all; clc;

input.AnimOn = 1;
input.PlotOn = 0;

% baseline_peak = run_simulation(1,input); %Peak with no arm swing
ti = 0.5 - 0.17; %Time range based on how long it takes for the arm to swing
tf = 0.5 + 0.17;
tis = ti:0.001:tf;
tis = 0.5 - 0.066;
% tis = tis(1);
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
xlabel('Time Difference (Shoulder-Hip) [s]')
ylabel('First CoM Height Peak [m]')