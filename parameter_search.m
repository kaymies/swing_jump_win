input.AnimOn = 0;
input.PlotOn = 0;

tis = 0.3:0.01:0.7;
peaks = zeros(1,length(tis));
for i = 1:length(tis)
    peaks(i) = run_simulation(tis(i),input);
    i
end