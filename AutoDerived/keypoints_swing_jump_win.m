function keypoints = keypoints_swing_jump_win(in1,in2)
%KEYPOINTS_SWING_JUMP_WIN
%    KEYPOINTS = KEYPOINTS_SWING_JUMP_WIN(IN1,IN2)


l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
le1 = in2(23,:);
le2 = in2(24,:);
tha = in1(2,:);
thh = in1(3,:);
ths = in1(4,:);
y = in1(1,:);
t2 = cos(tha);
t3 = cos(thh);
t4 = sin(tha);
t5 = sin(thh);
t10 = -le1;
t6 = l1.*t3;
t7 = l2.*t3;
t8 = l1.*t5;
t9 = l2.*t5;
t11 = l0+t10;
t12 = -t6;
t13 = -t8;
t14 = -t9;
keypoints = reshape([t7+t12+t2.*t11,t13+t14+y+t4.*t11,0.0,t7+t12+t2.*t10,t13+t14+y+t4.*t10,0.0,t7+t12+le2.*t3,t13+t14+y+le2.*t5,0.0,t7+t12,t13+t14+y,0.0,t7,t14+y,0.0,0.0,y,0.0,0.0,l3+y,0.0,l4.*sin(ths),l3+y-l4.*cos(ths),0.0],[3,8]);
