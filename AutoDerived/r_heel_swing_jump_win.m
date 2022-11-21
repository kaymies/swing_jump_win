function out1 = r_heel_swing_jump_win(in1,in2)
%R_HEEL_SWING_JUMP_WIN
%    OUT1 = R_HEEL_SWING_JUMP_WIN(IN1,IN2)

l1 = in2(2,:);
l2 = in2(3,:);
le1 = in2(23,:);
tha = in1(2,:);
thh = in1(3,:);
y = in1(1,:);
t2 = cos(thh);
t3 = sin(thh);
out1 = [-l1.*t2+l2.*t2-le1.*cos(tha);y-l1.*t3-l2.*t3-le1.*sin(tha)];
