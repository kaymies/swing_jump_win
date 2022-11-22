function Jt = J_toe_swing_jump_win(in1,in2)
%J_TOE_SWING_JUMP_WIN
%    JT = J_TOE_SWING_JUMP_WIN(IN1,IN2)


l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
le1 = in2(23,:);
tha = in1(2,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
t4 = -le1;
t5 = l0+t4;
Jt = reshape([0.0,1.0,-t5.*sin(tha),t5.*cos(tha),l1.*t3-l2.*t3,-l1.*t2-l2.*t2,0.0,0.0],[2,4]);
