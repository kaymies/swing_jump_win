function out1 = v_toe_swing_jump_win(in1,in2)
%V_TOE_SWING_JUMP_WIN
%    OUT1 = V_TOE_SWING_JUMP_WIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
<<<<<<< Updated upstream
%    06-Nov-2022 19:22:40
=======
%    10-Nov-2022 10:12:13
>>>>>>> Stashed changes

dtha = in1(6,:);
dy = in1(5,:);
l0 = in2(1,:);
tha = in1(2,:);
out1 = [-dtha.*l0.*sin(tha);dy+dtha.*l0.*cos(tha)];
