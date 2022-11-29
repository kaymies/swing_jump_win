function b = b_swing_jump_win(in1,in2,in3,in4)
%B_SWING_JUMP_WIN
%    B = B_SWING_JUMP_WIN(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    29-Nov-2022 14:50:49

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
taua = in2(1,:);
tauh = in2(2,:);
taus = in2(3,:);
tha = in1(2,:);
thh = in1(3,:);
ths = in1(4,:);
t2 = cos(thh);
t3 = cos(ths);
t4 = sin(thh);
t5 = sin(ths);
t6 = c2.^2;
t7 = c4.^2;
t8 = dthh.^2;
t9 = dths.^2;
t10 = l4.^2;
t19 = -dy;
t20 = -thh;
t11 = c1.*t2;
t12 = l1.*t2;
t13 = l2.*t2;
t14 = le2.*t2;
t15 = c1.*t4;
t16 = l1.*t4;
t17 = l2.*t4;
t18 = le2.*t4;
t21 = c4.*dths.*t5;
t22 = dths.*l4.*t5;
t23 = c2.*dthh.*t2;
t26 = t20+tha;
t24 = -t12;
t25 = -t13;
t27 = -t17;
t28 = cos(t26);
t29 = sin(t26);
t30 = dy+t21;
t31 = dy+t22;
t32 = -t23;
t33 = t11+t13;
t34 = t15+t17;
t35 = dy+t32;
t36 = c0.*t28;
t37 = le1.*t28;
t38 = c0.*t29;
t39 = le1.*t29;
t45 = dthh.*t33;
t46 = t11+t25;
t47 = t15+t27;
t40 = dtha.*t36;
t41 = dthh.*t36;
t42 = dtha.*t38;
t43 = dthh.*t38;
t48 = -t36;
t49 = -t39;
t54 = -t45;
t55 = t14+t37;
t63 = t16+t17+t38;
t67 = t13+t24+t36;
t68 = t16+t27+t38;
t44 = t42.*2.0;
t50 = -t40;
t51 = -t41;
t52 = -t42;
t53 = -t43;
t56 = dy+t54;
t57 = abs(t55);
t58 = sign(t55);
t59 = t18+t49;
t65 = dthh.*t63;
t66 = t12+t13+t48;
t70 = dthh.*t67;
t71 = dthh.*t68;
t60 = abs(t59);
t61 = sign(t59);
t62 = t57.^2;
t69 = dthh.*t66;
t72 = t40+t51;
t73 = t42+t53;
t74 = t52+t65;
t75 = t52+t71;
t76 = t50+t70;
t82 = t36.*(t42-t71).*-2.0;
t64 = t60.^2;
t77 = t19+t40+t69;
t78 = t62+t64;
t84 = t38.*t77.*2.0;
t79 = sqrt(t78);
t80 = 1.0./t79;
t81 = -t79;
t83 = ls0+t81;
et1 = tauh+(m0.*(t77.*(t42-t65).*2.0-(t40-t70).*(t42-t71).*2.0))./2.0-dthh.*((m1.*(t34.*t45.*-2.0+t34.*t56.*2.0+dthh.*t46.*t47.*4.0))./2.0+(m0.*(t63.*t77.*-2.0+t66.*(t42-t65).*2.0+t68.*(t40-t70).*2.0+t67.*(t42-t71).*2.0))./2.0+(m2.*(c2.*t4.*t35.*2.0+dthh.*t2.*t4.*t6.*2.0))./2.0)+(m2.*(c2.*dthh.*t4.*t35.*2.0+t2.*t4.*t6.*t8.*2.0))./2.0+(m1.*(dthh.*t34.*t56.*2.0+t8.*t46.*t47.*2.0))./2.0+(dtha.*m0.*(-t84+t66.*t73.*2.0+t68.*t72.*2.0+t36.*(t42-t71).*2.0))./2.0+g.*m1.*t33+g.*m0.*t66+c2.*g.*m2.*t2;
et2 = k.*t80.*t83.*(t57.*t58.*t59.*2.0-t55.*t60.*t61.*2.0).*(-1.0./2.0);
mt1 = [-g.*m0-g.*m1-g.*m2-g.*m3-g.*m4-g.*m5-dths.*(c4.*dths.*m4.*t3+dths.*l4.*m5.*t3)-dthh.*(m0.*(t44-t65.*2.0).*(-1.0./2.0)+dthh.*m1.*t34+c2.*dthh.*m2.*t4)+(dtha.*m0.*(t43.*2.0-t44))./2.0;taua-(m0.*(t73.*t77.*2.0-t72.*(t42-t71).*2.0))./2.0+(dtha.*m0.*(t82+t84+t36.*t73.*2.0-t38.*t72.*2.0))./2.0-(dthh.*m0.*(t82+t84+t36.*(t42-t65).*2.0-t38.*(t40-t70).*2.0))./2.0+g.*m0.*t36-(k.*t80.*t83.*(t39.*t57.*t58.*2.0+t37.*t60.*t61.*2.0))./2.0;et1+et2];
mt2 = [taus-dths.*((m4.*(c4.*t3.*t30.*2.0-dths.*t3.*t5.*t7.*2.0))./2.0+(m5.*(l4.*t3.*t31.*2.0-dths.*t3.*t5.*t10.*2.0))./2.0)+(m4.*(c4.*dths.*t3.*t30.*2.0-t3.*t5.*t7.*t9.*2.0))./2.0+(m5.*(dths.*l4.*t3.*t31.*2.0-t3.*t5.*t9.*t10.*2.0))./2.0-c4.*g.*m4.*t5-g.*l4.*m5.*t5];
b = [mt1;mt2];
