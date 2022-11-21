close all; clear all; clc;

input.AnimOn = 0;
input.PlotOn = 0;

tis = 0.3:0.001:0.65;
peaks = zeros(1,length(tis));
for i = 1:length(tis)
    peaks(i) = run_simulation(tis(i),input);
    i
end
figure();
plot(tis-0.5,peaks)
xlabel('Time Difference (Shoulder-Hip) [s]')
ylabel('First CoM Height Peak [m]')