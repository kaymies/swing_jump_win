function out1 = v_heel_swing_jump_win(in1,in2)
%V_HEEL_SWING_JUMP_WIN
%    OUT1 = V_HEEL_SWING_JUMP_WIN(IN1,IN2)

dtha = in1(6,:);
dthh = in1(7,:);
dy = in1(5,:);
l1 = in2(2,:);
l2 = in2(3,:);
le1 = in2(23,:);
tha = in1(2,:);
thh = in1(3,:);
t2 = cos(thh);
t3 = sin(thh);
out1 = [dthh.*(l1.*t3-l2.*t3)+dtha.*le1.*sin(tha);dy-dthh.*(l1.*t2+l2.*t2)-dtha.*le1.*cos(tha)];
