close all; clear all; clc;

input.AnimOn = 1;
input.PlotOn = 0;

% baseline_peak = run_simulation(1,input); %Peak with no arm swing

tis = 0.3:0.001:0.7;
% tis = 0.45;
tis = tis(22);
peaks = zeros(1,length(tis));
for i = 1:length(tis)
    out = run_simulation(tis(i),input);
    peaks(i) = out;
    i
end

%%
baseline_peak = run_simulation(1,input); %Peak with no arm swing
baseline_peak = 0;
figure();
plot(tis-0.5,peaks-baseline_peak)
xlabel('Time Difference (Shoulder-Hip) [s]')
ylabel('First CoM Height Peak [m]')