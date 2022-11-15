function [peak, t_peak] = find_first_peak(t,COM,tih)

[peaks, locs] = findpeaks(COM(2,:),t,"MinPeakHeight",0.18);

i = find(locs > tih, 1);

peak = peaks(i);
t_peak = locs(i);


% if (rand < 0.1)
%     figure
%     hold on
%     plot(t,COM(2,:));
%     scatter(locs,peaks);
%     scatter(t_peak,peak,'r*')
% end

if isempty(peak)

    peak = -100;

end

end