function f = objective(x,z0,p,tih,tis)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

  
    ctrl.tf = x(2);  
    ctrl.T = x(3:end);
    ctrl.tih = tih;
    ctrl.tis = tis;
%     z0,ctrl,p,tspan) [t0 tf] 
    [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,[0,x(1)]); % hybrid_simulation(z0,ctrl,p,tspan)  % ------------- CHECK - where get tspan? ------------------------

% Question 5 - Maximize COM height                                    
    COM_pos = COM_swing_jump_win(zout, p); % change to y height (torso position?)
    f = -max(COM_pos(2,:));

% Question 6 - minimize t_f
%     f = min(ctrl.tf);


% Question 7 - minimize sum of squared torques
%     f = min(sum(uout.^2));
end