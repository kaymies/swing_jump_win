function E = energy_swing_jump_win(in1,in2)
%ENERGY_SWING_JUMP_WIN
%    E = ENERGY_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    14-Nov-2022 15:31:49

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
t4 = sin(tha);
t5 = sin(thh);
t6 = sin(ths);
t7 = dthh.^2;
t8 = dths.^2;
t9 = t3.^2;
t10 = l2.*t2;
t11 = c1.*t5;
t12 = l1.*t5;
t13 = l2.*t5;
t14 = -t13;
et1 = (m5.*((dy+dths.*l4.*t6).^2+l4.^2.*t8.*t9))./2.0+(I1.*t7)./2.0+(I2.*t7)./2.0+(I4.*t8)./2.0+(m2.*((dy-c2.*dthh.*t2).^2+c2.^2.*t5.^2.*t7))./2.0+(I0.*dtha.^2)./2.0+(m0.*((dthh.*(t12+t14)-c0.*dtha.*t4).^2+(dy-dthh.*(t10+l1.*t2)+c0.*dtha.*cos(tha)).^2))./2.0+(dy.^2.*m3)./2.0+(m1.*(t7.*(t11+t14).^2+(dy-dthh.*(t10+c1.*t2)).^2))./2.0+(k.*(ls0-sqrt(le1.^2+le2.^2+le1.*le2.*cos(tha-thh).*2.0)).^2)./2.0+(m4.*((dy+c4.*dths.*t6).^2+c4.^2.*t8.*t9))./2.0;
et2 = g.*m2.*(y-c2.*t5)-g.*m1.*(t11+t13-y)+g.*m3.*(c3+y)-g.*m0.*(t12+t13-y-c0.*t4)+g.*m4.*(l3+y-c4.*t3)+g.*m5.*(l3+y-l4.*t3);
E = et1+et2;
