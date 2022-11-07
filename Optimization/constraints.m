function [cineq ceq] = constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().


    ctrl.tf = x(2);  % ------------- CHECK - where get ctrl? ------------------------
    ctrl.T = x(3:end);
    [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,[0,x(1)]);
    
    
    % Question 5  
%     cineq = [-min(zout(2,:)), max(zout(2, :))-pi/2];  
%     ceq = [x(2)- tout(indices(1))];  
    
%  Question 6 & 7  when velocity is zero and y com = 0.4 at the end  
    cineq = [-min(zout(2,:)), max(zout(2, :))-pi/2];  
    COM_pos = COM_jumping_leg(zout, p);
    [max_COM, ind] = max(COM_pos(2,:));
    % Find where COM y_dot = 0
    ceq = [0.4 - max_COM, COM_pos(4, ind)]; 


    
end