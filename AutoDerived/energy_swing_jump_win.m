function E = energy_swing_jump_win(in1,in2)
%ENERGY_SWING_JUMP_WIN
%    E = ENERGY_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
<<<<<<< Updated upstream
%    06-Nov-2022 19:22:36
=======
%    10-Nov-2022 10:12:09
>>>>>>> Stashed changes

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
t7 = dtha.^2;
t8 = dthh.^2;
t9 = l1.*t2;
t10 = l2.*t2;
t11 = c2.*t5;
t12 = l1.*t5;
t13 = l2.*t5;
t14 = -t13;
t15 = t9+t10;
t16 = dthh.*t15;
t17 = t12+t14;
t18 = dthh.*t17;
et1 = (I0.*t7)./2.0+(I1.*t8)./2.0+(I2.*t8)./2.0+(m4.*((dy+t16+c4.*dths.*t6).^2+(t18-c4.*dths.*t3).^2))./2.0+(m5.*((dy+t16+dths.*l4.*t6).^2+(t18-dths.*l4.*t3).^2))./2.0+(m1.*((dy+c1.*dthh.*t2).^2+c1.^2.*t5.^2.*t8))./2.0+(m3.*((dy+t16).^2+t8.*t17.^2))./2.0+(I4.*dths.^2)./2.0+(m0.*((dy+c0.*dtha.*cos(tha)).^2+c0.^2.*t4.^2.*t7))./2.0+(k.*(ls0-sqrt(le1.^2+le2.^2+le1.*le2.*cos(tha-thh).*2.0)).^2)./2.0+(m2.*(t8.*(t11-t12).^2+(dy+dthh.*(t9+c2.*t2)).^2))./2.0;
et2 = g.*m0.*(y+c0.*t4)+g.*m1.*(y+c1.*t5)+g.*m3.*(c3+t12+t13+y)+g.*m4.*(l3+t12+t13+y-c4.*t3)+g.*m5.*(l3+t12+t13+y-l4.*t3)+g.*m2.*(t11+t12+y);
E = et1+et2;
