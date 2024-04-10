function [zdot,Jz,Ju] = quad_sys(in1,in2,m,J,g,d)
%QUAD_SYS
%    [ZDOT,JZ,JU] = QUAD_SYS(IN1,IN2,M,J,G,D)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Sep-2023 16:35:40

f1 = in2(1,:);
f2 = in2(2,:);
q = in1(6,:);
theta = in1(3,:);
xdot = in1(4,:);
ydot = in1(5,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = f1+f2;
t5 = 1.0./J;
t6 = 1.0./m;
t7 = d.*t5;
t8 = t2.*t6;
t9 = t3.*t6;
t10 = -t9;
t11 = t4.*t8;
t12 = t4.*t9;
t13 = t4.*t10;
zdot = [xdot;ydot;q;t13;-g+t11;-t7.*(f1-f2)];
if nargout > 1
    Jz = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t11,t13,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],[6,6]);
end
if nargout > 2
    Ju = reshape([0.0,0.0,0.0,t10,t8,-t7,0.0,0.0,0.0,t10,t8,t7],[6,2]);
end
