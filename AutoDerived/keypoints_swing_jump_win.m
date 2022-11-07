function keypoints = keypoints_swing_jump_win(in1,in2)
%KEYPOINTS_SWING_JUMP_WIN
%    KEYPOINTS = KEYPOINTS_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Nov-2022 17:09:53

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
t6 = l1.*t3;
t7 = l2.*t3;
t8 = l1.*t5;
t9 = l2.*t5;
t10 = -t7;
t11 = t6+t10;
keypoints = reshape([l0.*t2,y+l0.*t4,0.0,-le1.*t2,y-le1.*t4,0.0,le2.*t3,y+le2.*t5,0.0,0.0,y,0.0,t6,t8+y,0.0,t11,t8+t9+y,0.0,t11,l3+t8+t9+y,0.0,t11+l4.*sin(ths),l3+t8+t9+y-l4.*cos(ths),0.0],[3,8]);
