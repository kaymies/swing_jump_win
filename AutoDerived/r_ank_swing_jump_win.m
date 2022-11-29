function out1 = r_ank_swing_jump_win(in1,in2)
%R_ANK_SWING_JUMP_WIN
%    OUT1 = R_ANK_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    29-Nov-2022 14:50:51

l1 = in2(2,:);
l2 = in2(3,:);
thh = in1(3,:);
y = in1(1,:);
t2 = cos(thh);
t3 = sin(thh);
out1 = [-l1.*t2+l2.*t2;y-l1.*t3-l2.*t3];
