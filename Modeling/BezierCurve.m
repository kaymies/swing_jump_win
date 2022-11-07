function u = BezierCurve(ctrl_pt, t)

%% Samuel Gollob, Oct 2023
%Implementation of the explicit definition of a Bezier curve, according to
%the equation described in the Wikipedia page for Bezier curves.
%Assumes that 0 <= t <= 1, else returns zero

if t < 0 || t > 1
u = 0;

else

n = length(ctrl_pt) - 1; % Max index of points, assuming indexing at zero (i.e.: the order of the curve)
u = 0;
    for i = 0 : n
        coeff = nchoosek(n,i);
        P_curr = ctrl_pt(i+1);
        u = (coeff * (1-t)^(n-i) * t^i * P_curr) + u; % Explicit definition of Bezier curve as a sum
    end
end

end