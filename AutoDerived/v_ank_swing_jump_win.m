function out1 = v_ank_swing_jump_win(in1,in2)
%V_ANK_SWING_JUMP_WIN
%    OUT1 = V_ANK_SWING_JUMP_WIN(IN1,IN2)

dthh = in1(7,:);
dy = in1(5,:);
l1 = in2(2,:);
l2 = in2(3,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
out1 = [dthh.*(l1.*t3-l2.*t3);dy-dthh.*(l1.*t2+l2.*t2)];
