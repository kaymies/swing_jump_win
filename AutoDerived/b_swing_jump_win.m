function b = b_swing_jump_win(in1,in2,in3,in4)
%B_SWING_JUMP_WIN
%    B = B_SWING_JUMP_WIN(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
<<<<<<< Updated upstream
%    06-Nov-2022 19:22:38
=======
%    10-Nov-2022 10:12:11
>>>>>>> Stashed changes

Fy = in3(1,:);
Fy2 = in3(2,:);
c0 = in4(6,:);
c1 = in4(7,:);
c2 = in4(8,:);
c4 = in4(10,:);
dtha = in1(6,:);
dthh = in1(7,:);
dths = in1(8,:);
dy = in1(5,:);
g = in4(22,:);
k = in4(25,:);
l0 = in4(1,:);
l1 = in4(2,:);
l2 = in4(3,:);
l4 = in4(5,:);
le1 = in4(23,:);
le2 = in4(24,:);
ls0 = in4(26,:);
m0 = in4(11,:);
m1 = in4(12,:);
m2 = in4(13,:);
m3 = in4(14,:);
m4 = in4(15,:);
m5 = in4(16,:);
tauh = in2(2,:);
taus = in2(3,:);
tha = in1(2,:);
thh = in1(3,:);
ths = in1(4,:);
t2 = cos(tha);
t3 = cos(thh);
t4 = cos(ths);
t5 = sin(tha);
t6 = sin(thh);
t7 = sin(ths);
t8 = c0.^2;
t9 = c1.^2;
t10 = dtha.^2;
t11 = dthh.^2;
t12 = le1.^2;
t13 = le2.^2;
t20 = -thh;
t14 = c2.*t3;
t15 = l1.*t3;
t16 = l2.*t3;
t17 = c2.*t6;
t18 = l1.*t6;
t19 = l2.*t6;
t21 = dths.*l4.*t4;
t22 = c4.*dths.*t7;
t23 = dths.*l4.*t7;
t24 = c0.*dtha.*t2;
t25 = c1.*dthh.*t3;
t26 = c4.*dths.*t4;
t29 = t20+tha;
t27 = -t15;
t28 = -t16;
t30 = -t18;
t31 = -t19;
t32 = -t21;
t33 = cos(t29);
t34 = sin(t29);
t35 = dy+t24;
t36 = dy+t25;
t37 = -t26;
t38 = t14+t15;
t39 = t15+t16;
t40 = t17+t18;
t41 = t18+t19;
t42 = le1.*le2.*t33.*2.0;
t43 = dthh.*t38;
t44 = dthh.*t39;
t45 = t14+t27;
t46 = t15+t28;
t47 = t17+t30;
t48 = t18+t31;
t49 = dy+t43;
t50 = dy+t44;
t51 = dthh.*t48;
t52 = t12+t13+t42;
t55 = t41.*t44.*2.0;
t53 = t22+t50;
t54 = t23+t50;
t56 = t37+t51;
t57 = t32+t51;
t58 = sqrt(t52);
t60 = t46.*t51.*2.0;
t59 = conj(t58);
t61 = -t60;
t62 = -t59;
t63 = 1.0./t59;
t64 = ls0+t62;
t65 = k.*le1.*le2.*t34.*t63.*t64;
et1 = t65+tauh+dthh.*((m3.*(t55+t41.*t50.*2.0-t46.*t51.*4.0))./2.0+(m2.*(t40.*t43.*2.0+t40.*t49.*2.0-dthh.*t45.*t47.*4.0))./2.0+(m5.*(t55+t61+t41.*t54.*2.0+t46.*(t21-t51).*2.0))./2.0+(m4.*(t55+t61+t41.*t53.*2.0+t46.*(t26-t51).*2.0))./2.0+(m1.*(c1.*t6.*t36.*2.0-dthh.*t3.*t6.*t9.*2.0))./2.0)-(m1.*(c1.*dthh.*t6.*t36.*2.0-t3.*t6.*t9.*t11.*2.0))./2.0-(m2.*(dthh.*t40.*t49.*2.0-t11.*t45.*t47.*2.0))./2.0-(m3.*(dthh.*t41.*t50.*2.0-t11.*t46.*t48.*2.0))./2.0-dths.*((m5.*(t21.*t39.*2.0+t23.*t48.*2.0))./2.0+(m4.*(t26.*t39.*2.0+t22.*t48.*2.0))./2.0);
et2 = m5.*(dthh.*t41.*t54.*2.0+dthh.*t46.*(t21-t51).*2.0).*(-1.0./2.0)-(m4.*(dthh.*t41.*t53.*2.0+dthh.*t46.*(t26-t51).*2.0))./2.0-g.*m2.*t38-g.*m3.*t39-g.*m4.*t39-g.*m5.*t39-c1.*g.*m1.*t3;
mt1 = [Fy+Fy2-g.*m0-g.*m1-g.*m2-g.*m3-g.*m4-g.*m5-dths.*(m5.*t21+m4.*t26)+dthh.*(dthh.*m2.*t40+dthh.*m3.*t41+dthh.*m4.*t41+dthh.*m5.*t41+c1.*dthh.*m1.*t6)+c0.*m0.*t5.*t10,-t65-(m0.*(c0.*dtha.*t5.*t35.*2.0-t2.*t5.*t8.*t10.*2.0))./2.0+(dtha.*m0.*(c0.*t5.*t35.*2.0-dtha.*t2.*t5.*t8.*2.0))./2.0+Fy.*l0.*t2-c0.*g.*m0.*t2,et1+et2];
mt2 = [taus-dths.*((m4.*(c4.*t4.*t53.*2.0-c4.*t7.*(t26-t51).*2.0))./2.0+(m5.*(l4.*t4.*t54.*2.0-l4.*t7.*(t21-t51).*2.0))./2.0)+dthh.*((m4.*(c4.*dthh.*t7.*t41.*2.0+c4.*dthh.*t4.*t46.*2.0))./2.0+(m5.*(dthh.*l4.*t7.*t41.*2.0+dthh.*l4.*t4.*t46.*2.0))./2.0)+(m5.*(t21.*t54.*2.0-t23.*(t21-t51).*2.0))./2.0+(m4.*(t26.*t53.*2.0-t22.*(t26-t51).*2.0))./2.0-c4.*g.*m4.*t7-g.*l4.*m5.*t7];
b = reshape([mt1,mt2],4,1);
