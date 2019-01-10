//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SteadyStateAcceleration_dCOM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//

// Include Files
#include "rt_nonfinite.h"
#include "VelocityEstimator.h"
#include "SteadyStateAcceleration_dCOM.h"

// Function Definitions

//
// function dAcceleration_dCOM = SteadyStateAcceleration_dCOM(COM_X,COM_Y,COM_Z,Jk,Mb,Mk,g,q1,q2,q3,q4,rk)
// STEADYSTATEACCELERATION_DCOM
//     DACCELERATION_DCOM = STEADYSTATEACCELERATION_DCOM(COM_X,COM_Y,COM_Z,JK,MB,MK,G,Q1,Q2,Q3,Q4,RK)
// Arguments    : float COM_X
//                float COM_Y
//                float COM_Z
//                float Jk
//                float Mb
//                float Mk
//                float g
//                float q1
//                float q2
//                float q3
//                float q4
//                float rk
//                float dAcceleration_dCOM[4]
// Return Type  : void
//
__attribute__((optimize("O3"))) void SteadyStateAcceleration_dCOM(float COM_X, float COM_Y, float COM_Z, float
  Jk, float Mb, float Mk, float g, float q1, float q2, float q3, float q4, float
  rk, float dAcceleration_dCOM[4])
{
  float t2;
  float t3;
  float t4;
  float t5;
  float t6;
  float t7;
  float t8;
  float t9;
  float t10;
  float t28;
  float t37;
  float t38;
  float t39;
  float t40;
  float t41;
  float t42;
  float t43;
  float t44;
  float t45;
  float t46;
  float t47;
  float t48;
  float t49;
  float t50;
  float t51;
  float t52;
  float t53;
  float t54;
  float t55;
  float t56;
  float t57;
  float t58;
  float t59;
  float t60;
  float t61;
  float t62;
  float t63;
  float t64;
  float t65;
  float t66;
  float t67;
  float t68;
  float t69;
  float t70;
  float t71;
  float f10;
  float x[4];
  int i4;
  int i5;
  int i6;
  int i7;

  //     This function was generated by the Symbolic Math Toolbox version 8.1.
  //     16-Nov-2018 12:04:30
  // 'SteadyStateAcceleration_dCOM:8' t2 = rk.^2;
  t2 = rk * rk;

  // 'SteadyStateAcceleration_dCOM:9' t3 = q1.^2;
  t3 = q1 * q1;

  // 'SteadyStateAcceleration_dCOM:10' t4 = q2.^2;
  t4 = q2 * q2;

  // 'SteadyStateAcceleration_dCOM:11' t5 = q3.^2;
  t5 = q3 * q3;

  // 'SteadyStateAcceleration_dCOM:12' t6 = q4.^2;
  t6 = q4 * q4;

  // 'SteadyStateAcceleration_dCOM:13' t7 = t3.^2;
  t7 = t3 * t3;

  // 'SteadyStateAcceleration_dCOM:14' t8 = t4.^2;
  t8 = t4 * t4;

  // 'SteadyStateAcceleration_dCOM:15' t9 = t5.^2;
  t9 = t5 * t5;

  // 'SteadyStateAcceleration_dCOM:16' t10 = t6.^2;
  t10 = t6 * t6;

  // 'SteadyStateAcceleration_dCOM:17' t11 = Mb.*t2;
  // 'SteadyStateAcceleration_dCOM:18' t12 = Mk.*t2;
  // 'SteadyStateAcceleration_dCOM:19' t13 = COM_Z.*Mb.*rk.*t7;
  // 'SteadyStateAcceleration_dCOM:20' t14 = COM_Z.*Mb.*rk.*t10;
  // 'SteadyStateAcceleration_dCOM:21' t15 = COM_Z.*Mb.*rk.*t3.*t6.*2.0;
  // 'SteadyStateAcceleration_dCOM:22' t16 = COM_Y.*Mb.*q1.*q2.*rk.*t4.*2.0;
  // 'SteadyStateAcceleration_dCOM:23' t17 = COM_Y.*Mb.*q1.*q2.*rk.*t3.*2.0;
  // 'SteadyStateAcceleration_dCOM:24' t18 = COM_X.*Mb.*q2.*q4.*rk.*t6.*2.0;
  // 'SteadyStateAcceleration_dCOM:25' t19 = COM_X.*Mb.*q2.*q4.*rk.*t4.*2.0;
  // 'SteadyStateAcceleration_dCOM:26' t20 = COM_Y.*Mb.*q3.*q4.*rk.*t6.*2.0;
  // 'SteadyStateAcceleration_dCOM:27' t21 = COM_Y.*Mb.*q3.*q4.*rk.*t5.*2.0;
  // 'SteadyStateAcceleration_dCOM:28' t22 = COM_X.*Mb.*q2.*q4.*rk.*t3.*2.0;
  // 'SteadyStateAcceleration_dCOM:29' t23 = COM_Y.*Mb.*q1.*q2.*rk.*t5.*2.0;
  // 'SteadyStateAcceleration_dCOM:30' t24 = COM_Y.*Mb.*q1.*q2.*rk.*t6.*2.0;
  // 'SteadyStateAcceleration_dCOM:31' t25 = COM_X.*Mb.*q2.*q4.*rk.*t5.*2.0;
  // 'SteadyStateAcceleration_dCOM:32' t26 = COM_Y.*Mb.*q3.*q4.*rk.*t3.*2.0;
  // 'SteadyStateAcceleration_dCOM:33' t27 = COM_Y.*Mb.*q3.*q4.*rk.*t4.*2.0;
  // 'SteadyStateAcceleration_dCOM:34' t30 = COM_Z.*Mb.*rk.*t8;
  // 'SteadyStateAcceleration_dCOM:35' t31 = COM_Z.*Mb.*rk.*t9;
  // 'SteadyStateAcceleration_dCOM:36' t32 = COM_Z.*Mb.*rk.*t4.*t5.*2.0;
  // 'SteadyStateAcceleration_dCOM:37' t33 = COM_X.*Mb.*q1.*q3.*rk.*t5.*2.0;
  // 'SteadyStateAcceleration_dCOM:38' t34 = COM_X.*Mb.*q1.*q3.*rk.*t3.*2.0;
  // 'SteadyStateAcceleration_dCOM:39' t35 = COM_X.*Mb.*q1.*q3.*rk.*t4.*2.0;
  // 'SteadyStateAcceleration_dCOM:40' t36 = COM_X.*Mb.*q1.*q3.*rk.*t6.*2.0;
  // 'SteadyStateAcceleration_dCOM:41' t28 = Jk+t11+t12+t13+t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27-t30-t31-t32-t33-t34-t35-t36; 
  t28 = (((((((((((((((((((((((Jk + Mb * t2) + Mk * t2) + COM_Z * Mb * rk * t7)
    + COM_Z * Mb * rk * t10) + COM_Z * Mb * rk * t3 * t6 * 2.0F) + COM_Y * Mb *
    q1 * q2 * rk * t4 * 2.0F) + COM_Y * Mb * q1 * q2 * rk * t3 * 2.0F) + COM_X *
                        Mb * q2 * q4 * rk * t6 * 2.0F) + COM_X * Mb * q2 * q4 *
                       rk * t4 * 2.0F) + COM_Y * Mb * q3 * q4 * rk * t6 * 2.0F)
                     + COM_Y * Mb * q3 * q4 * rk * t5 * 2.0F) + COM_X * Mb * q2 *
                    q4 * rk * t3 * 2.0F) + COM_Y * Mb * q1 * q2 * rk * t5 * 2.0F)
                  + COM_Y * Mb * q1 * q2 * rk * t6 * 2.0F) + COM_X * Mb * q2 *
                 q4 * rk * t5 * 2.0F) + COM_Y * Mb * q3 * q4 * rk * t3 * 2.0F) +
               COM_Y * Mb * q3 * q4 * rk * t4 * 2.0F) - COM_Z * Mb * rk * t8) -
             COM_Z * Mb * rk * t9) - COM_Z * Mb * rk * t4 * t5 * 2.0F) - COM_X *
           Mb * q1 * q3 * rk * t5 * 2.0F) - COM_X * Mb * q1 * q3 * rk * t3 *
          2.0F) - COM_X * Mb * q1 * q3 * rk * t4 * 2.0F) - COM_X * Mb * q1 * q3 *
    rk * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:42' t29 = 1.0./t28.^2;
  t28 = 1.0F / (t28 * t28);

  // 'SteadyStateAcceleration_dCOM:43' t37 = Jk.*q1.*q4;
  t37 = Jk * q1 * q4;

  // 'SteadyStateAcceleration_dCOM:44' t38 = Mb.*q1.*q4.*t2;
  t38 = Mb * q1 * q4 * t2;

  // 'SteadyStateAcceleration_dCOM:45' t39 = Mk.*q1.*q4.*t2;
  t39 = Mk * q1 * q4 * t2;

  // 'SteadyStateAcceleration_dCOM:46' t40 = COM_Z.*Mb.*q1.*q4.*rk.*t3.*t6.*2.0; 
  t40 = COM_Z * Mb * q1 * q4 * rk * t3 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:47' t41 = COM_Z.*Mb.*q2.*q3.*rk.*t4.*t5.*2.0; 
  t41 = COM_Z * Mb * q2 * q3 * rk * t4 * t5 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:48' t42 = COM_Z.*Mb.*q1.*q4.*rk.*t10;
  t42 = COM_Z * Mb * q1 * q4 * rk * t10;

  // 'SteadyStateAcceleration_dCOM:49' t43 = COM_Z.*Mb.*q2.*q3.*rk.*t9;
  t43 = COM_Z * Mb * q2 * q3 * rk * t9;

  // 'SteadyStateAcceleration_dCOM:50' t44 = COM_Z.*Mb.*q1.*q4.*rk.*t7;
  t44 = COM_Z * Mb * q1 * q4 * rk * t7;

  // 'SteadyStateAcceleration_dCOM:51' t45 = COM_Z.*Mb.*q2.*q3.*rk.*t8;
  t45 = COM_Z * Mb * q2 * q3 * rk * t8;

  // 'SteadyStateAcceleration_dCOM:52' t46 = COM_Z.*Mb.*q2.*q3.*rk.*t7;
  t46 = COM_Z * Mb * q2 * q3 * rk * t7;

  // 'SteadyStateAcceleration_dCOM:53' t47 = COM_Z.*Mb.*q1.*q4.*rk.*t8;
  t47 = COM_Z * Mb * q1 * q4 * rk * t8;

  // 'SteadyStateAcceleration_dCOM:54' t48 = COM_Z.*Mb.*q1.*q4.*rk.*t9;
  t48 = COM_Z * Mb * q1 * q4 * rk * t9;

  // 'SteadyStateAcceleration_dCOM:55' t49 = COM_Z.*Mb.*q2.*q3.*rk.*t10;
  t49 = COM_Z * Mb * q2 * q3 * rk * t10;

  // 'SteadyStateAcceleration_dCOM:56' t50 = COM_Z.*Mb.*q2.*q3.*rk.*t3.*t5.*2.0; 
  t50 = COM_Z * Mb * q2 * q3 * rk * t3 * t5 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:57' t51 = COM_Z.*Mb.*q2.*q3.*rk.*t3.*t4.*2.0; 
  t51 = COM_Z * Mb * q2 * q3 * rk * t3 * t4 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:58' t52 = COM_Z.*Mb.*q1.*q4.*rk.*t4.*t6.*2.0; 
  t52 = COM_Z * Mb * q1 * q4 * rk * t4 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:59' t53 = COM_Z.*Mb.*q1.*q4.*rk.*t3.*t4.*2.0; 
  t53 = COM_Z * Mb * q1 * q4 * rk * t3 * t4 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:60' t54 = COM_Z.*Mb.*q1.*q4.*rk.*t5.*t6.*2.0; 
  t54 = COM_Z * Mb * q1 * q4 * rk * t5 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:61' t55 = COM_Z.*Mb.*q1.*q4.*rk.*t3.*t5.*2.0; 
  t55 = COM_Z * Mb * q1 * q4 * rk * t3 * t5 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:62' t56 = COM_Z.*Mb.*q2.*q3.*rk.*t5.*t6.*2.0; 
  t56 = COM_Z * Mb * q2 * q3 * rk * t5 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:63' t57 = COM_Z.*Mb.*q2.*q3.*rk.*t4.*t6.*2.0; 
  t57 = COM_Z * Mb * q2 * q3 * rk * t4 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:64' t58 = COM_Z.*Mb.*q1.*q4.*rk.*t4.*t5.*2.0; 
  t58 = COM_Z * Mb * q1 * q4 * rk * t4 * t5 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:65' t59 = COM_Z.*Mb.*q2.*q3.*rk.*t3.*t6.*2.0; 
  t59 = COM_Z * Mb * q2 * q3 * rk * t3 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:66' t60 = Jk.*t3;
  t60 = Jk * t3;

  // 'SteadyStateAcceleration_dCOM:67' t61 = Jk.*t4;
  t61 = Jk * t4;

  // 'SteadyStateAcceleration_dCOM:68' t62 = Mb.*t2.*t3;
  t62 = Mb * t2 * t3;

  // 'SteadyStateAcceleration_dCOM:69' t63 = Mb.*t2.*t4;
  t63 = Mb * t2 * t4;

  // 'SteadyStateAcceleration_dCOM:70' t64 = Mk.*t2.*t3;
  t64 = Mk * t2 * t3;

  // 'SteadyStateAcceleration_dCOM:71' t65 = Mk.*t2.*t4;
  t65 = Mk * t2 * t4;

  // 'SteadyStateAcceleration_dCOM:72' t66 = COM_Z.*Mb.*rk.*t3.*t7;
  t66 = COM_Z * Mb * rk * t3 * t7;

  // 'SteadyStateAcceleration_dCOM:73' t67 = COM_Z.*Mb.*rk.*t5.*t9;
  t67 = COM_Z * Mb * rk * t5 * t9;

  // 'SteadyStateAcceleration_dCOM:74' t68 = COM_Z.*Mb.*rk.*t4.*t9;
  t68 = COM_Z * Mb * rk * t4 * t9;

  // 'SteadyStateAcceleration_dCOM:75' t69 = COM_Z.*Mb.*rk.*t6.*t7;
  t69 = COM_Z * Mb * rk * t6 * t7;

  // 'SteadyStateAcceleration_dCOM:76' t70 = COM_Z.*Mb.*rk.*t3.*t4.*t5.*2.0;
  t70 = COM_Z * Mb * rk * t3 * t4 * t5 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:77' t71 = COM_Z.*Mb.*rk.*t3.*t5.*t6.*2.0;
  t71 = COM_Z * Mb * rk * t3 * t5 * t6 * 2.0F;

  // 'SteadyStateAcceleration_dCOM:78' dAcceleration_dCOM = reshape([Mb.*g.*rk.*t29.*(t60+t61+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71-Jk.*t5-Jk.*t6-Mb.*t2.*t5-Mb.*t2.*t6-Mk.*t2.*t5-Mk.*t2.*t6-COM_Z.*Mb.*rk.*t3.*t8+COM_Z.*Mb.*rk.*t4.*t7+COM_Z.*Mb.*rk.*t3.*t9.*3.0-COM_Z.*Mb.*rk.*t4.*t8+COM_Z.*Mb.*rk.*t5.*t7.*3.0-COM_Z.*Mb.*rk.*t3.*t10-COM_Z.*Mb.*rk.*t5.*t8-COM_Z.*Mb.*rk.*t4.*t10.*3.0-COM_Z.*Mb.*rk.*t6.*t8.*3.0-COM_Z.*Mb.*rk.*t5.*t10+COM_Z.*Mb.*rk.*t6.*t9-COM_Z.*Mb.*rk.*t6.*t10+COM_Y.*Mb.*q1.*q2.*rk.*t7.*2.0+COM_Y.*Mb.*q1.*q2.*rk.*t8.*2.0+COM_Y.*Mb.*q1.*q2.*rk.*t9.*2.0+COM_Y.*Mb.*q1.*q2.*rk.*t10.*2.0-COM_Y.*Mb.*q3.*q4.*rk.*t7.*2.0-COM_Y.*Mb.*q3.*q4.*rk.*t8.*2.0-COM_Y.*Mb.*q3.*q4.*rk.*t9.*2.0-COM_Y.*Mb.*q3.*q4.*rk.*t10.*2.0-COM_Z.*Mb.*rk.*t3.*t4.*t6.*2.0-COM_Z.*Mb.*rk.*t4.*t5.*t6.*2.0+COM_Y.*Mb.*q1.*q2.*rk.*t3.*t4.*4.0+COM_Y.*Mb.*q1.*q2.*rk.*t3.*t5.*4.0+COM_Y.*Mb.*q1.*q2.*rk.*t3.*t6.*4.0+COM_Y.*Mb.*q1.*q2.*rk.*t4.*t5.*4.0+COM_Y.*Mb.*q1.*q2.*rk.*t4.*t6.*4.0+COM_Y.*Mb.*q1.*q2.*rk.*t5.*t6.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t3.*t4.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t3.*t5.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t3.*t6.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t4.*t5.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t4.*t6.*4.0-COM_Y.*Mb.*q3.*q4.*rk.*t5.*t6.*4.0),Mb.*g.*rk.*t29.*(t37+t38+t39+t40-t41+t42-t43+t44-t45-t46+t47+t48-t49-t50-t51+t52+t53+t54+t55-t56-t57+t58-t59+Jk.*q2.*q3+Mb.*q2.*q3.*t2+Mk.*q2.*q3.*t2+COM_Y.*Mb.*q1.*q3.*rk.*t7+COM_Y.*Mb.*q1.*q3.*rk.*t8+COM_Y.*Mb.*q1.*q3.*rk.*t9+COM_Y.*Mb.*q2.*q4.*rk.*t7+COM_Y.*Mb.*q1.*q3.*rk.*t10+COM_Y.*Mb.*q2.*q4.*rk.*t8+COM_Y.*Mb.*q2.*q4.*rk.*t9+COM_Y.*Mb.*q2.*q4.*rk.*t10+COM_Y.*Mb.*q1.*q3.*rk.*t3.*t4.*2.0+COM_Y.*Mb.*q1.*q3.*rk.*t3.*t5.*2.0+COM_Y.*Mb.*q1.*q3.*rk.*t3.*t6.*2.0+COM_Y.*Mb.*q1.*q3.*rk.*t4.*t5.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t3.*t4.*2.0+COM_Y.*Mb.*q1.*q3.*rk.*t4.*t6.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t3.*t5.*2.0+COM_Y.*Mb.*q1.*q3.*rk.*t5.*t6.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t3.*t6.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t4.*t5.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t4.*t6.*2.0+COM_Y.*Mb.*q2.*q4.*rk.*t5.*t6.*2.0).*2.0,Mb.*g.*rk.*t29.*(t37+t38+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59-Jk.*q2.*q3-Mb.*q2.*q3.*t2-Mk.*q2.*q3.*t2+COM_X.*Mb.*q1.*q2.*rk.*t7+COM_X.*Mb.*q1.*q2.*rk.*t8+COM_X.*Mb.*q1.*q2.*rk.*t9+COM_X.*Mb.*q1.*q2.*rk.*t10-COM_X.*Mb.*q3.*q4.*rk.*t7-COM_X.*Mb.*q3.*q4.*rk.*t8-COM_X.*Mb.*q3.*q4.*rk.*t9-COM_X.*Mb.*q3.*q4.*rk.*t10+COM_X.*Mb.*q1.*q2.*rk.*t3.*t4.*2.0+COM_X.*Mb.*q1.*q2.*rk.*t3.*t5.*2.0+COM_X.*Mb.*q1.*q2.*rk.*t3.*t6.*2.0+COM_X.*Mb.*q1.*q2.*rk.*t4.*t5.*2.0+COM_X.*Mb.*q1.*q2.*rk.*t4.*t6.*2.0+COM_X.*Mb.*q1.*q2.*rk.*t5.*t6.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t3.*t4.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t3.*t5.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t3.*t6.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t4.*t5.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t4.*t6.*2.0-COM_X.*Mb.*q3.*q4.*rk.*t5.*t6.*2.0).*-2.0,-Mb.*g.*rk.*t29.*(-t60+t61-t62+t63-t64+t65-t66+t67+t68-t69-t70+t71-Jk.*t5+Jk.*t6-Mb.*t2.*t5+Mb.*t2.*t6-Mk.*t2.*t5+Mk.*t2.*t6-COM_Z.*Mb.*rk.*t3.*t8.*3.0-COM_Z.*Mb.*rk.*t4.*t7.*3.0+COM_Z.*Mb.*rk.*t3.*t9-COM_Z.*Mb.*rk.*t4.*t8-COM_Z.*Mb.*rk.*t5.*t7+COM_Z.*Mb.*rk.*t3.*t10-COM_Z.*Mb.*rk.*t5.*t8+COM_Z.*Mb.*rk.*t4.*t10-COM_Z.*Mb.*rk.*t6.*t8+COM_Z.*Mb.*rk.*t5.*t10.*3.0+COM_Z.*Mb.*rk.*t6.*t9.*3.0+COM_Z.*Mb.*rk.*t6.*t10+COM_X.*Mb.*q1.*q3.*rk.*t7.*2.0+COM_X.*Mb.*q1.*q3.*rk.*t8.*2.0+COM_X.*Mb.*q1.*q3.*rk.*t9.*2.0+COM_X.*Mb.*q2.*q4.*rk.*t7.*2.0+COM_X.*Mb.*q1.*q3.*rk.*t10.*2.0+COM_X.*Mb.*q2.*q4.*rk.*t8.*2.0+COM_X.*Mb.*q2.*q4.*rk.*t9.*2.0+COM_X.*Mb.*q2.*q4.*rk.*t10.*2.0-COM_Z.*Mb.*rk.*t3.*t4.*t6.*2.0+COM_Z.*Mb.*rk.*t4.*t5.*t6.*2.0+COM_X.*Mb.*q1.*q3.*rk.*t3.*t4.*4.0+COM_X.*Mb.*q1.*q3.*rk.*t3.*t5.*4.0+COM_X.*Mb.*q1.*q3.*rk.*t3.*t6.*4.0+COM_X.*Mb.*q1.*q3.*rk.*t4.*t5.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t3.*t4.*4.0+COM_X.*Mb.*q1.*q3.*rk.*t4.*t6.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t3.*t5.*4.0+COM_X.*Mb.*q1.*q3.*rk.*t5.*t6.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t3.*t6.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t4.*t5.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t4.*t6.*4.0+COM_X.*Mb.*q2.*q4.*rk.*t5.*t6.*4.0)],[2,2]); 
  f10 = (((t60 + t61) + t62) + t63) + t64;
  x[0] = Mb * g * rk * t28 * (((((((((((((((((((((((((((((((((((((((((((((((f10
    + t65) + t66) + t67) + t68) + t69) + t70) + t71) - Jk * t5) - Jk * t6) - Mb *
    t2 * t5) - Mb * t2 * t6) - Mk * t2 * t5) - Mk * t2 * t6) - COM_Z * Mb * rk *
    t3 * t8) + COM_Z * Mb * rk * t4 * t7) + COM_Z * Mb * rk * t3 * t9 * 3.0F) -
    COM_Z * Mb * rk * t4 * t8) + COM_Z * Mb * rk * t5 * t7 * 3.0F) - COM_Z * Mb *
    rk * t3 * t10) - COM_Z * Mb * rk * t5 * t8) - COM_Z * Mb * rk * t4 * t10 *
    3.0F) - COM_Z * Mb * rk * t6 * t8 * 3.0F) - COM_Z * Mb * rk * t5 * t10) +
    COM_Z * Mb * rk * t6 * t9) - COM_Z * Mb * rk * t6 * t10) + COM_Y * Mb * q1 *
    q2 * rk * t7 * 2.0F) + COM_Y * Mb * q1 * q2 * rk * t8 * 2.0F) + COM_Y * Mb *
    q1 * q2 * rk * t9 * 2.0F) + COM_Y * Mb * q1 * q2 * rk * t10 * 2.0F) - COM_Y *
    Mb * q3 * q4 * rk * t7 * 2.0F) - COM_Y * Mb * q3 * q4 * rk * t8 * 2.0F) -
    COM_Y * Mb * q3 * q4 * rk * t9 * 2.0F) - COM_Y * Mb * q3 * q4 * rk * t10 *
    2.0F) - COM_Z * Mb * rk * t3 * t4 * t6 * 2.0F) - COM_Z * Mb * rk * t4 * t5 *
    t6 * 2.0F) + COM_Y * Mb * q1 * q2 * rk * t3 * t4 * 4.0F) + COM_Y * Mb * q1 *
    q2 * rk * t3 * t5 * 4.0F) + COM_Y * Mb * q1 * q2 * rk * t3 * t6 * 4.0F) +
    COM_Y * Mb * q1 * q2 * rk * t4 * t5 * 4.0F) + COM_Y * Mb * q1 * q2 * rk * t4
    * t6 * 4.0F) + COM_Y * Mb * q1 * q2 * rk * t5 * t6 * 4.0F) - COM_Y * Mb * q3
    * q4 * rk * t3 * t4 * 4.0F) - COM_Y * Mb * q3 * q4 * rk * t3 * t5 * 4.0F) -
    COM_Y * Mb * q3 * q4 * rk * t3 * t6 * 4.0F) - COM_Y * Mb * q3 * q4 * rk * t4
    * t5 * 4.0F) - COM_Y * Mb * q3 * q4 * rk * t4 * t6 * 4.0F) - COM_Y * Mb * q3
    * q4 * rk * t5 * t6 * 4.0F);
  x[1] = Mb * g * rk * t28 * (((((((((((((((((((((((((((((((((((((((((((((t37 +
    t38) + t39) + t40) - t41) + t42) - t43) + t44) - t45) - t46) + t47) + t48) -
    t49) - t50) - t51) + t52) + t53) + t54) + t55) - t56) - t57) + t58) - t59) +
    Jk * q2 * q3) + Mb * q2 * q3 * t2) + Mk * q2 * q3 * t2) + COM_Y * Mb * q1 *
    q3 * rk * t7) + COM_Y * Mb * q1 * q3 * rk * t8) + COM_Y * Mb * q1 * q3 * rk *
    t9) + COM_Y * Mb * q2 * q4 * rk * t7) + COM_Y * Mb * q1 * q3 * rk * t10) +
    COM_Y * Mb * q2 * q4 * rk * t8) + COM_Y * Mb * q2 * q4 * rk * t9) + COM_Y *
    Mb * q2 * q4 * rk * t10) + COM_Y * Mb * q1 * q3 * rk * t3 * t4 * 2.0F) +
    COM_Y * Mb * q1 * q3 * rk * t3 * t5 * 2.0F) + COM_Y * Mb * q1 * q3 * rk * t3
    * t6 * 2.0F) + COM_Y * Mb * q1 * q3 * rk * t4 * t5 * 2.0F) + COM_Y * Mb * q2
    * q4 * rk * t3 * t4 * 2.0F) + COM_Y * Mb * q1 * q3 * rk * t4 * t6 * 2.0F) +
    COM_Y * Mb * q2 * q4 * rk * t3 * t5 * 2.0F) + COM_Y * Mb * q1 * q3 * rk * t5
    * t6 * 2.0F) + COM_Y * Mb * q2 * q4 * rk * t3 * t6 * 2.0F) + COM_Y * Mb * q2
    * q4 * rk * t4 * t5 * 2.0F) + COM_Y * Mb * q2 * q4 * rk * t4 * t6 * 2.0F) +
    COM_Y * Mb * q2 * q4 * rk * t5 * t6 * 2.0F) * 2.0F;
  x[2] = Mb * g * rk * t28 * (((((((((((((((((((((((((((((((((((((((((((((t37 +
    t38) + t39) + t40) + t41) + t42) + t43) + t44) + t45) + t46) + t47) + t48) +
    t49) + t50) + t51) + t52) + t53) + t54) + t55) + t56) + t57) + t58) + t59) -
    Jk * q2 * q3) - Mb * q2 * q3 * t2) - Mk * q2 * q3 * t2) + COM_X * Mb * q1 *
    q2 * rk * t7) + COM_X * Mb * q1 * q2 * rk * t8) + COM_X * Mb * q1 * q2 * rk *
    t9) + COM_X * Mb * q1 * q2 * rk * t10) - COM_X * Mb * q3 * q4 * rk * t7) -
    COM_X * Mb * q3 * q4 * rk * t8) - COM_X * Mb * q3 * q4 * rk * t9) - COM_X *
    Mb * q3 * q4 * rk * t10) + COM_X * Mb * q1 * q2 * rk * t3 * t4 * 2.0F) +
    COM_X * Mb * q1 * q2 * rk * t3 * t5 * 2.0F) + COM_X * Mb * q1 * q2 * rk * t3
    * t6 * 2.0F) + COM_X * Mb * q1 * q2 * rk * t4 * t5 * 2.0F) + COM_X * Mb * q1
    * q2 * rk * t4 * t6 * 2.0F) + COM_X * Mb * q1 * q2 * rk * t5 * t6 * 2.0F) -
    COM_X * Mb * q3 * q4 * rk * t3 * t4 * 2.0F) - COM_X * Mb * q3 * q4 * rk * t3
    * t5 * 2.0F) - COM_X * Mb * q3 * q4 * rk * t3 * t6 * 2.0F) - COM_X * Mb * q3
    * q4 * rk * t4 * t5 * 2.0F) - COM_X * Mb * q3 * q4 * rk * t4 * t6 * 2.0F) -
    COM_X * Mb * q3 * q4 * rk * t5 * t6 * 2.0F) * -2.0F;
  f10 = (((-t60 + t61) - t62) + t63) - t64;
  x[3] = -Mb * g * rk * t28 * (((((((((((((((((((((((((((((((((((((((((((((((f10
    + t65) - t66) + t67) + t68) - t69) - t70) + t71) - Jk * t5) + Jk * t6) - Mb *
    t2 * t5) + Mb * t2 * t6) - Mk * t2 * t5) + Mk * t2 * t6) - COM_Z * Mb * rk *
    t3 * t8 * 3.0F) - COM_Z * Mb * rk * t4 * t7 * 3.0F) + COM_Z * Mb * rk * t3 *
    t9) - COM_Z * Mb * rk * t4 * t8) - COM_Z * Mb * rk * t5 * t7) + COM_Z * Mb *
    rk * t3 * t10) - COM_Z * Mb * rk * t5 * t8) + COM_Z * Mb * rk * t4 * t10) -
    COM_Z * Mb * rk * t6 * t8) + COM_Z * Mb * rk * t5 * t10 * 3.0F) + COM_Z * Mb
    * rk * t6 * t9 * 3.0F) + COM_Z * Mb * rk * t6 * t10) + COM_X * Mb * q1 * q3 *
    rk * t7 * 2.0F) + COM_X * Mb * q1 * q3 * rk * t8 * 2.0F) + COM_X * Mb * q1 *
    q3 * rk * t9 * 2.0F) + COM_X * Mb * q2 * q4 * rk * t7 * 2.0F) + COM_X * Mb *
    q1 * q3 * rk * t10 * 2.0F) + COM_X * Mb * q2 * q4 * rk * t8 * 2.0F) + COM_X *
    Mb * q2 * q4 * rk * t9 * 2.0F) + COM_X * Mb * q2 * q4 * rk * t10 * 2.0F) -
    COM_Z * Mb * rk * t3 * t4 * t6 * 2.0F) + COM_Z * Mb * rk * t4 * t5 * t6 *
    2.0F) + COM_X * Mb * q1 * q3 * rk * t3 * t4 * 4.0F) + COM_X * Mb * q1 * q3 *
    rk * t3 * t5 * 4.0F) + COM_X * Mb * q1 * q3 * rk * t3 * t6 * 4.0F) + COM_X *
    Mb * q1 * q3 * rk * t4 * t5 * 4.0F) + COM_X * Mb * q2 * q4 * rk * t3 * t4 *
    4.0F) + COM_X * Mb * q1 * q3 * rk * t4 * t6 * 4.0F) + COM_X * Mb * q2 * q4 *
    rk * t3 * t5 * 4.0F) + COM_X * Mb * q1 * q3 * rk * t5 * t6 * 4.0F) + COM_X *
    Mb * q2 * q4 * rk * t3 * t6 * 4.0F) + COM_X * Mb * q2 * q4 * rk * t4 * t5 *
    4.0F) + COM_X * Mb * q2 * q4 * rk * t4 * t6 * 4.0F) + COM_X * Mb * q2 * q4 *
    rk * t5 * t6 * 4.0F);
  i4 = 0;
  i5 = 0;
  i6 = 0;
  for (i7 = 0; i7 < 4; i7++) {
    dAcceleration_dCOM[i5 + (i4 << 1)] = x[i6];
    i4++;
    if (i4 > 1) {
      i4 = 0;
      i5++;
    }

    i6++;
  }
}

//
// File trailer for SteadyStateAcceleration_dCOM.cpp
//
// [EOF]
//
