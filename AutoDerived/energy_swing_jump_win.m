function E = energy_swing_jump_win(in1,in2)
%ENERGY_SWING_JUMP_WIN
%    E = ENERGY_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Nov-2022 14:43:53

I0 = in2(17,:);
I1 = in2(18,:);
I2 = in2(19,:);
I4 = in2(21,:);
c0 = in2(6,:);
c1 = in2(7,:);
c2 = in2(8,:);
c3 = in2(9,:);
c4 = in2(10,:);
dtha = in1(6,:);
dthh = in1(7,:);
dths = in1(8,:);
dy = in1(5,:);
g = in2(22,:);
k = in2(25,:);
l1 = in2(2,:);
l2 = in2(3,:);
l3 = in2(4,:);
l4 = in2(5,:);
le1 = in2(23,:);
le2 = in2(24,:);
ls0 = in2(26,:);
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
t2 = cos(thh);
t3 = cos(ths);
t4 = sin(thh);
t5 = sin(ths);
t6 = dthh.^2;
t7 = dths.^2;
t13 = -thh;
t14 = -y;
t8 = t3.^2;
t9 = l2.*t2;
t10 = c1.*t4;
t11 = l1.*t4;
t12 = l2.*t4;
t15 = t13+tha;
t16 = -t12;
t17 = cos(t15);
t18 = sin(t15);
t19 = c0.*t18;
et1 = (m5.*((dy+dths.*l4.*t5).^2+l4.^2.*t7.*t8))./2.0+(I1.*t6)./2.0+(I2.*t6)./2.0+(I4.*t7)./2.0+(m2.*((dy-c2.*dthh.*t2).^2+c2.^2.*t4.^2.*t6))./2.0+(k.*(ls0-sqrt(le1.^2+le2.^2+le1.*le2.*t17.*2.0)).^2)./2.0+(I0.*dtha.^2)./2.0+(dy.^2.*m3)./2.0+(m1.*(t6.*(t10+t16).^2+(dy-dthh.*(t9+c1.*t2)).^2))./2.0+(m0.*((-dy+dthh.*(t9-c0.*t17+l1.*t2)+c0.*dtha.*t17).^2+(dtha.*t19-dthh.*(t11+t16+t19)).^2))./2.0+(m4.*((dy+c4.*dths.*t5).^2+c4.^2.*t7.*t8))./2.0;
et2 = g.*m2.*(y-c2.*t4)-g.*m0.*(t11+t12+t14+t19)+g.*m3.*(c3+y)+g.*m4.*(l3+y-c4.*t3)+g.*m5.*(l3+y-l4.*t3)-g.*m1.*(t10+t12+t14);
E = et1+et2;
