function Jt = J_toe_swing_jump_win(in1,in2)
%J_toe_swing_jump_win
%    Jt = J_toe_swing_jump_win(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Nov-2022 16:28:39

l0 = in2(1,:);
tha = in1(2,:);
Jt = reshape([0.0,1.0,-l0.*sin(tha),l0.*cos(tha),0.0,0.0,0.0,0.0],[2,4]);
