tis = 0.3:0.01:0.7;
peak = zeros(length(tis));
for i = 1:length(tis)
    peak(i) = run_simulation(tis(i));
end