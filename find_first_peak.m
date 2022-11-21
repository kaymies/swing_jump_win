function [peak_max, t_max] = find_first_peak(t,y,tih)

% [peaks, locs] = findpeaks(y,t,"MinPeakHeight",0.18);
t_window = 0.08;
[peaks, locs] = findpeaks(y,t,"MinPeakHeight",0.17);

i = find(locs > tih);
peaks = peaks(i);
locs = locs(i);

peak = peaks(1);
t_peak = locs(1);

% [sort_peaks, I] = sort(peaks,'descend');
% sort_locs = locs(I);

t_max = t_peak;
peak_max = peak;
t_curr = t_peak;
lim = length(i);
j = 2;
while ((t_curr - t_peak) < t_window) && (j <= lim)
    t_curr = locs(j);
    curr_peak = peaks(j);
    if curr_peak > peak_max
        peak_max = curr_peak;
        t_max = t_curr;
    end
    j = j+1;
end



if (rand < 0.1)
    figure
    hold on
    plot(t,y);
    scatter(locs,peaks);
    scatter(t_max,peak_max,'r*')
end

end