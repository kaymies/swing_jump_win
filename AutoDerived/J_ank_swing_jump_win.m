function Ja = J_ank_swing_jump_win(in1,in2)
%J_ANK_SWING_JUMP_WIN
%    JA = J_ANK_SWING_JUMP_WIN(IN1,IN2)

l1 = in2(2,:);
l2 = in2(3,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
Ja = reshape([0.0,1.0,0.0,0.0,l1.*t3-l2.*t3,-l1.*t2-l2.*t2,0.0,0.0],[2,4]);
