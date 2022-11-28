function out1 = v_toe_swing_jump_win(in1,in2)
%V_TOE_SWING_JUMP_WIN
%    OUT1 = V_TOE_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Nov-2022 13:53:30

dtha = in1(6,:);
dthh = in1(7,:);
dy = in1(5,:);
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
out1 = [dthh.*(l1.*t3-l2.*t3+t6.*t9)-dtha.*t6.*t9;dy-dthh.*(l1.*t2+l2.*t2-t6.*t8)-dtha.*t6.*t8];
