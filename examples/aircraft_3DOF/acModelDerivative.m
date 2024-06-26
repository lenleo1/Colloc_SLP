function [xdot,Jx,Ju,y,Jyx,Jyu] = acModelDerivative(in1,in2)
%ACMODELDERIVATIVE
%    [XDOT,JX,JU,Y,JYX,JYU] = ACMODELDERIVATIVE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-Jan-2021 21:22:08

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
t2 = cos(u3);
t3 = cos(x5);
t4 = cos(x6);
t5 = sin(u3);
t6 = sin(x5);
t7 = sin(x6);
t8 = u1.*4.0;
t9 = x4.^2;
t10 = 1.0./x4;
t11 = 1.0./t4;
t12 = t3.*t4.*x4;
t13 = t4.*t6.*x4;
t14 = t8+1.0./5.0;
t15 = t14.^2;
t16 = t15./2.5e+1;
t17 = t16+3.0./1.0e+2;
xdot = [t12;t13;-t7.*x4;t7.*(-9.81e+2./1.0e+2)+u2.*2.943-t9.*t17.*9.625e-4;t5.*t11.*t14.*x4.*9.625e-4;t4.*t10.*(-9.81e+2./1.0e+2)+t2.*t14.*x4.*9.625e-4];
if nargout > 1
    Jx = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t3.*t4,t4.*t6,-t7,t17.*x4.*(-1.925e-3),t5.*t11.*t14.*9.625e-4,t2.*t14.*9.625e-4+(t4.*(9.81e+2./1.0e+2))./t9,-t13,t12,0.0,0.0,0.0,0.0,-t3.*t7.*x4,-t6.*t7.*x4,-t4.*x4,t4.*(-9.81e+2./1.0e+2),t5.*t7.*t11.^2.*t14.*x4.*9.625e-4,t7.*t10.*(9.81e+2./1.0e+2)],[6,6]);
end
if nargout > 2
    Ju = reshape([0.0,0.0,0.0,t9.*(u1.*(3.2e+1./2.5e+1)+8.0./1.25e+2).*(-9.625e-4),t5.*t11.*x4.*3.85e-3,t2.*x4.*3.85e-3,0.0,0.0,0.0,2.943,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t11.*t14.*x4.*9.625e-4,t5.*t14.*x4.*(-9.625e-4)],[6,3]);
end
if nargout > 3
    y = t9.*t14.*9.811416921508665e-5;
end
if nargout > 4
    Jyx = [0.0,0.0,0.0,t14.*x4.*1.962283384301733e-4,0.0,0.0];
end
if nargout > 5
    Jyu = [t9.*3.924566768603466e-4,0.0,0.0];
end
