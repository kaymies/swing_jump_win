function COM = COM_swing_jump_win(in1,in2)
%COM_SWING_JUMP_WIN
%    COM = COM_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    31-Oct-2022 16:57:46

c0 = in2(6,:);
c1 = in2(7,:);
c2 = in2(8,:);
c3 = in2(9,:);
c4 = in2(10,:);
dtha = in1(6,:);
dthh = in1(7,:);
dths = in1(8,:);
dy = in1(5,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
m0 = in2(11,:);
m1 = in2(12,:);
m2 = in2(13,:);
m3 = in2(14,:);
m4 = in2(15,:);
m5 = in2(16,:);
tha = in1(2,:);
thh = in1(3,:);
ths = in1(4,:);
y = in1(1,:);
t2 = cos(tha);
t3 = cos(thh);
t4 = cos(ths);
t5 = sin(tha);
t6 = sin(thh);
t7 = sin(ths);
t17 = m0+m1+m2+m3+m4+m5;
t8 = c2.*t3;
t9 = l1.*t3;
t10 = l2.*t3;
t11 = c2.*t6;
t12 = l1.*t6;
t13 = l2.*t6;
t14 = c1.*m1.*t3;
t19 = 1.0./t17;
t15 = -t10;
t16 = -t13;
t18 = t9+t10;
t20 = t12+t16;
COM = [t19.*(t14+m3.*(t9+t15)+m4.*(t9+t15+c4.*t7)+m5.*(t9+t15+l4.*t7)-m2.*(t8-t9)+c0.*m0.*t2);t19.*(m4.*(l3+t12+t13+y-c4.*t4)+m5.*(l3+t12+t13+y-l4.*t4)+m2.*(t11+t12+y)+m0.*(y+c0.*t5)+m1.*(y+c1.*t6)+m3.*(c3+t12+t13+y));-dthh.*t19.*(m3.*t20+m4.*t20+m5.*t20-m2.*(t11-t12)+c1.*m1.*t6)+dths.*t19.*(c4.*m4.*t4+l4.*m5.*t4)-c0.*dtha.*m0.*t5.*t19;dy+dthh.*t19.*(t14+m2.*(t8+t9)+m3.*t18+m4.*t18+m5.*t18)+dths.*t19.*(c4.*m4.*t7+l4.*m5.*t7)+c0.*dtha.*m0.*t2.*t19];