function u = BezierCurve(ctrl_pt, t)

n = length(ctrl_pt);
u = 0;
    for i = 1:n
%         u = 1; % compute return value. Write your code instead of 1.
%         u = u + (1-t)*((1-t)*ctrl_pt.T[1) + t*ctrl_pt.T(2)) + t*((1-t)*ctrl_pt.T(2) + t*ctrl.pt(3))
%         u = u+ (factorial(3-1)/(factorial(i-1)*factorial(3-i)))*(t^(i-1))*((1-t)^(3-i))*ctrl_pt(i);
        u = u+ (factorial(n-1)/(factorial(i-1)*factorial(n-i)))*(t^(i-1))*((1-t)^(n-i))*ctrl_pt(i);
    end
end