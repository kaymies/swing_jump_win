function keypoints = keypoints_swing_jump_win(in1,in2)
%KEYPOINTS_SWING_JUMP_WIN
%    KEYPOINTS = KEYPOINTS_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    31-Oct-2022 16:57:46

l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
tha = in1(2,:);
thh = in1(3,:);
ths = in1(4,:);
y = in1(1,:);
t2 = cos(thh);
t3 = sin(thh);
t4 = l1.*t2;
t5 = l2.*t2;
t6 = l1.*t3;
t7 = l2.*t3;
t8 = -t5;
t9 = t4+t8;
keypoints = reshape([l0.*cos(tha),y+l0.*sin(tha),0.0,0.0,y,0.0,t4,t6+y,0.0,t9,t6+t7+y,0.0,t9,l3+t6+t7+y,0.0,t9+l4.*sin(ths),l3+t6+t7+y-l4.*cos(ths),0.0],[3,6]);