function Ja = J_ank_swing_jump_win(in1,in2)
%J_ank_swing_jump_win
%    Ja = J_ank_swing_jump_win(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Nov-2022 14:43:55

l1 = in2(2,:);
l2 = in2(3,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
Ja = reshape([0.0,1.0,0.0,0.0,l1.*t3-l2.*t3,-l1.*t2-l2.*t2,0.0,0.0],[2,4]);
