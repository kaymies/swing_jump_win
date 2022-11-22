function out1 = r_toe_swing_jump_win(in1,in2)
%R_TOE_SWING_JUMP_WIN
%    OUT1 = R_TOE_SWING_JUMP_WIN(IN1,IN2)


l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
le1 = in2(23,:);
tha = in1(2,:);
thh = in1(3,:);
y = in1(1,:);
t2 = cos(thh);
t3 = sin(thh);
t4 = -le1;
t5 = l0+t4;
out1 = [-l1.*t2+l2.*t2+t5.*cos(tha);y-l1.*t3-l2.*t3+t5.*sin(tha)];
