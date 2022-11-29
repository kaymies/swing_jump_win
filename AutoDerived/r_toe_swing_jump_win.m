function out1 = r_toe_swing_jump_win(in1,in2)
%R_TOE_SWING_JUMP_WIN
%    OUT1 = R_TOE_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    29-Nov-2022 14:50:50

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
t5 = -thh;
t6 = l0+t4;
t7 = t5+tha;
out1 = [-l1.*t2+l2.*t2+t6.*cos(t7);y-l1.*t3-l2.*t3-t6.*sin(t7)];
