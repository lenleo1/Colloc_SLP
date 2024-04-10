function [dz,Jz,Ju] = quad_6dof_sys(in1,in2)
%QUAD_6DOF_SYS
%    [DZ,JZ,JU] = QUAD_6DOF_SYS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    15-Sep-2023 13:19:06

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
z4 = in1(4,:);
z5 = in1(5,:);
z6 = in1(6,:);
z7 = in1(7,:);
z8 = in1(8,:);
z9 = in1(9,:);
z10 = in1(10,:);
z11 = in1(11,:);
z12 = in1(12,:);
t2 = cos(z6);
t3 = sin(z4);
t4 = cos(z4);
t5 = sin(z5);
t6 = sin(z6);
t7 = u1.^2;
t8 = t7.*5.000000000000001;
t9 = u2.^2;
t10 = t9.*5.000000000000001;
t11 = u3.^2;
t12 = t11.*5.000000000000001;
t13 = u4.^2;
t14 = t13.*5.000000000000001;
t15 = t8+t10+t12+t14;
t16 = t3.*t6;
t17 = t2.*t4.*t5;
t18 = t16+t17;
t19 = cos(z5);
t20 = z10.*z12;
t21 = t11.*2.000000000000001e1;
t34 = t7.*2.000000000000001e1;
t22 = t20+t21-t34;
t23 = t2.*t3;
t37 = t4.*t5.*t6;
t24 = t23-t37;
t25 = t7.*8.000000000000001e-1;
t26 = t9.*8.000000000000001e-1;
t27 = t11.*8.000000000000001e-1;
t28 = t13.*8.000000000000001e-1;
t29 = t25-t26+t27-t28;
t30 = z11.*z12;
t31 = t13.*2.000000000000001e1;
t33 = t9.*2.000000000000001e1;
t32 = t30+t31-t33;
t35 = t4.*t6;
t42 = t2.*t3.*t5;
t36 = t35-t42;
t38 = t2.*t4;
t39 = t3.*t5.*t6;
t40 = t38+t39;
t41 = t15.*t18;
t43 = t22.*t40;
t44 = t24.*t29;
t45 = -t18.*t29-t22.*t36-t2.*t19.*t32;
dz = [z7;z8;z9;z10;z11;z12;t41;-t15.*t24;t4.*t15.*t19-9.81e2./1.0e2;t45;t43+t44-t6.*t19.*t32;t5.*t32+t3.*t19.*t22-t4.*t19.*t29];
if nargout > 1
    Jz = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t15.*t36,-t15.*t40,-t3.*t15.*t19,t18.*t22-t29.*t36,-t22.*t24+t29.*t40,t4.*t19.*t22+t3.*t19.*t29,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t4.*t15.*t19,t4.*t6.*t15.*t19,-t4.*t5.*t15,t2.*t5.*t32+t2.*t3.*t19.*t22-t2.*t4.*t19.*t29,t5.*t6.*t32+t3.*t6.*t19.*t22-t4.*t6.*t19.*t29,t19.*t32-t3.*t5.*t22+t4.*t5.*t29,0.0,0.0,0.0,0.0,0.0,0.0,t15.*t24,t41,0.0,-t43-t44+t6.*t19.*t32,t45,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,-t36.*z12,t40.*z12,t3.*t19.*z12,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,-t2.*t19.*z12,-t6.*t19.*z12,t5.*z12,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-t36.*z10-t2.*t19.*z11,t40.*z10-t6.*t19.*z11,t5.*z11+t3.*t19.*z10],[12,12]);
end
if nargout > 2
    Ju = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t18.*u1.*1.0e1,t24.*u1.*(-1.0e1),t4.*t19.*u1.*1.0e1,t18.*u1.*(-1.6)+t36.*u1.*4.000000000000001e1,t24.*u1.*1.6-t40.*u1.*4.000000000000001e1,t3.*t19.*u1.*(-4.000000000000001e1)-t4.*t19.*u1.*1.6,0.0,0.0,0.0,0.0,0.0,0.0,t18.*u2.*1.0e1,t24.*u2.*(-1.0e1),t4.*t19.*u2.*1.0e1,t18.*u2.*1.6+t2.*t19.*u2.*4.000000000000001e1,t24.*u2.*(-1.6)+t6.*t19.*u2.*4.000000000000001e1,t5.*u2.*(-4.000000000000001e1)+t4.*t19.*u2.*1.6,0.0,0.0,0.0,0.0,0.0,0.0,t18.*u3.*1.0e1,t24.*u3.*(-1.0e1),t4.*t19.*u3.*1.0e1,t18.*u3.*(-1.6)-t36.*u3.*4.000000000000001e1,t24.*u3.*1.6+t40.*u3.*4.000000000000001e1,t3.*t19.*u3.*4.000000000000001e1-t4.*t19.*u3.*1.6,0.0,0.0,0.0,0.0,0.0,0.0,t18.*u4.*1.0e1,t24.*u4.*(-1.0e1),t4.*t19.*u4.*1.0e1,t18.*u4.*1.6-t2.*t19.*u4.*4.000000000000001e1,t24.*u4.*(-1.6)-t6.*t19.*u4.*4.000000000000001e1,t5.*u4.*4.000000000000001e1+t4.*t19.*u4.*1.6],[12,4]);
end
