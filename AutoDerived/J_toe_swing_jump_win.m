function Jt = J_toe_swing_jump_win(in1,in2)
%J_toe_swing_jump_win
%    Jt = J_toe_swing_jump_win(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Nov-2022 14:43:55

l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
le1 = in2(23,:);
tha = in1(2,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
t4 = -le1;
t5 = -thh;
t6 = l0+t4;
t7 = t5+tha;
t8 = cos(t7);
t9 = sin(t7);
t10 = t6.*t8;
t11 = t6.*t9;
Jt = reshape([0.0,1.0,-t11,-t10,t11+l1.*t3-l2.*t3,t10-l1.*t2-l2.*t2,0.0,0.0],[2,4]);
