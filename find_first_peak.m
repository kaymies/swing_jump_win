function [peak_max, t_max] = find_first_peak(t,y,COM,tih,tis)

% [peaks, locs] = findpeaks(y,t,"MinPeakHeight",0.18);
t_window = 0.2;
y_ground = 0.168; %y height for extended leg touching ground

%Cut off y before tih
i = find(t > tih);
y_new = y(i);
t_new = t(i);

%Cut off y after touches ground after first jump
airtime = y_new > y_ground;
found = false;
j_end = 1;
for j = 1:length(y_new)
    if ~found
        if airtime(j)
            found = true;
            j_end = j;
        end
    else
        j_end = j;
        if ~airtime(j)
            break;
        end
    end
end

y_new = y_new(1:j_end);
t_new = t_new(1:j_end);

% figure
% hold on
% plot(t,y)
% plot(t_new,y_new)

% Find the peaks and mark first peak
[peaks, locs] = findpeaks(y_new,t_new,"MinPeakHeight",y_ground);

[peak_max, I] = max(peaks);

if isempty(peak_max)
    peak_max = y_ground;
    t_max = tih;
else
    peak_max = peak_max(1);
    t_max = locs(I(1));
end




% 
% % if (rand < 0.1)
% %     figure
% %     plot(t,COM(2,:))
%     hold on
%     plot(t,y);
%     scatter(locs,peaks);
%     scatter(t_max,peak_max,'r*')
%     title(num2str(tis))
% % end

end