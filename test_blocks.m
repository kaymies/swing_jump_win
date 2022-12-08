
clear all; close all; clc;
dths = 10; %shoulder angular velocity
thsi = -0.2; %Initial shoulder angle
thsf = 4; %Final shoulder angle
tis = 1.5;

ramp = @(t) t .* heaviside(t);
tfs = tis + (thsf - thsi)/dths;
thsfun = @(t) dths * (ramp(t - tis) - ramp(t - tfs)) + thsi;
dthsfun = @(t) dths * (heaviside(t - tis) - heaviside(t - tfs));


figure
yyaxis left
fplot(thsfun,[0,3])
yyaxis right
fplot(dthsfun,[0,3])