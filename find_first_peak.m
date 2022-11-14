function [peak, t_peak] = find_first_peak(t,COM)

[peaks, locs] = findpeaks(COM(2,:),t,"MinPeakHeight",0.18);

peak = peaks(1);
t_peak = locs(1);

% figure
% hold on
% plot(t,COM(2,:));
% scatter(locs,peaks);

end