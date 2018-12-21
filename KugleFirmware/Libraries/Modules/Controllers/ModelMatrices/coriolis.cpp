//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: coriolis.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 20-Nov-2018 22:55:49
//

// Include Files
#include "rt_nonfinite.h"
#include "coriolis.h"

// Function Definitions

//
// function C = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw)
// CORIOLIS
//     C = CORIOLIS(COM_X,COM_Y,COM_Z,JBX,JBY,JBZ,JW,MB,BETA,DQ1,DQ2,DQ3,DQ4,DX,DY,Q1,Q2,Q3,Q4,RK,RW)
// Arguments    : float COM_X
//                float COM_Y
//                float COM_Z
//                float Jbx
//                float Jby
//                float Jbz
//                float Jw
//                float Mb
//                double beta
//                float dq1
//                float dq2
//                float dq3
//                float dq4
//                float dx
//                float dy
//                float q1
//                float q2
//                float q3
//                float q4
//                float rk
//                float rw
//                float C[36]
// Return Type  : void
//
void coriolis(float COM_X, float COM_Y, float COM_Z, float Jbx, float Jby, float
              Jbz, float Jw, float Mb, double beta, float dq1, float dq2, float
              dq3, float dq4, float dx, float dy, float q1, float q2, float q3,
              float q4, float rk, float rw, float C[36])
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
  float t13;
  float t16;
  float t25;
  float t26;
  float t27;
  float t28;
  float t29;
  float t30;
  float t31;
  float t32;
  float t33;
  float t34;
  float t35;
  float t36;
  float t39;
  float t42;
  float t44;
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
  float t72;
  float t73;
  float t74;
  float t75;
  float t76;
  float t77;
  float t78;
  float t79;
  float t80;
  float t81;
  float t82;
  float t83;
  float t84;
  float t85;
  float t86;
  float t87;
  float t88;
  float t89;
  float t90;
  float t91;
  float t92;
  float t93;
  float t94;
  float t95;
  float t96;
  float t97;
  float t98;
  float t99;
  float t100;
  float t101;
  float t102;
  float t103;
  float t104;
  float t105;
  float t106;
  float t107;
  float t108;
  float t109;
  float t110;
  float t111;
  float t112;
  float t113;
  float t114;
  float t115;
  float t116;
  float t117;
  float t118;
  float t125;
  float t130;
  float t131;
  float t132;
  float t133;
  float t134;
  float t135;
  float t137;
  float t138;
  float t139;
  float t140;
  float t141;
  float t149;
  float t157;
  float t160;
  float t168;
  float t171;
  float t172;
  float t174;
  float t175;
  float t176;
  float t179;
  float t182;
  float t183;
  float t185;
  float t188;
  float t190;
  float t273;
  float t192;
  float t195;
  float t196;
  float t202;
  float x[36];
  int i4;
  int i5;
  int i6;
  int i7;

  //     This function was generated by the Symbolic Math Toolbox version 8.1.
  //     12-Nov-2018 22:44:43
  // 'coriolis:8' t2 = q1.^2;
  t2 = q1 * q1;

  // 'coriolis:9' t3 = q2.^2;
  t3 = q2 * q2;

  // 'coriolis:10' t4 = q3.^2;
  t4 = q3 * q3;

  // 'coriolis:11' t5 = q4.^2;
  t5 = q4 * q4;

  // 'coriolis:12' t6 = 1.0./rw.^2;
  t6 = 1.0F / (rw * rw);

  // 'coriolis:13' t7 = rw.^2;
  t7 = rw * rw;

  // 'coriolis:14' t8 = COM_Z.*Mb.*t7.*4.0;
  t8 = COM_Z * Mb * t7 * 4.0F;

  // 'coriolis:15' t9 = Jw.*rk.*t4.*3.0;
  t9 = Jw * rk * t4 * 3.0F;

  // 'coriolis:16' t10 = Jw.*rk.*t5.*3.0;
  t10 = Jw * rk * t5 * 3.0F;

  // 'coriolis:17' t11 = COM_X.*Mb.*t7.*2.0;
  // 'coriolis:18' t12 = Jw.*q1.*q3.*rk.*3.0;
  // 'coriolis:19' t17 = Jw.*q2.*q4.*rk.*3.0;
  // 'coriolis:20' t13 = t11+t12-t17;
  t13 = (COM_X * Mb * t7 * 2.0F + Jw * q1 * q3 * rk * 3.0F) - Jw * q2 * q4 * rk *
    3.0F;

  // 'coriolis:21' t14 = Jw.*q1.*q2.*rk.*6.0;
  // 'coriolis:22' t15 = Jw.*q3.*q4.*rk.*6.0;
  // 'coriolis:23' t16 = t14+t15-COM_Y.*Mb.*t7.*4.0;
  t16 = (Jw * q1 * q2 * rk * 6.0F + Jw * q3 * q4 * rk * 6.0F) - COM_Y * Mb * t7 *
    4.0F;

  // 'coriolis:24' t18 = dq2.*q3.*t2;
  // 'coriolis:25' t19 = dq3.*q2.*t2;
  // 'coriolis:26' t20 = dq1.*q4.*t3;
  // 'coriolis:27' t21 = dq4.*q1.*t3;
  // 'coriolis:28' t22 = dq1.*q1.*q2.*q3.*2.0;
  // 'coriolis:29' t23 = dq2.*q1.*q2.*q4.*2.0;
  // 'coriolis:30' t24 = t18+t19+t20+t21+t22+t23-dq1.*q4.*t4-dq4.*q1.*t4-dq2.*q3.*t5-dq3.*q2.*t5-dq3.*q1.*q3.*q4.*2.0-dq4.*q2.*q3.*q4.*2.0; 
  // 'coriolis:31' t25 = Jw.*t6.*t24.*3.0;
  t25 = Jw * t6 * (((((((((((dq2 * q3 * t2 + dq3 * q2 * t2) + dq1 * q4 * t3) +
    dq4 * q1 * t3) + dq1 * q1 * q2 * q3 * 2.0F) + dq2 * q1 * q2 * q4 * 2.0F) -
                        dq1 * q4 * t4) - dq4 * q1 * t4) - dq2 * q3 * t5) - dq3 *
                     q2 * t5) - dq3 * q1 * q3 * q4 * 2.0F) - dq4 * q2 * q3 * q4 *
                   2.0F) * 3.0F;

  // 'coriolis:32' t26 = dq1.*q1.*t2;
  t26 = dq1 * q1 * t2;

  // 'coriolis:33' t27 = dq2.*q2.*t3;
  t27 = dq2 * q2 * t3;

  // 'coriolis:34' t28 = dq3.*q3.*t4;
  t28 = dq3 * q3 * t4;

  // 'coriolis:35' t29 = dq4.*q4.*t5;
  t29 = dq4 * q4 * t5;

  // 'coriolis:36' t30 = dq1.*q1.*t5;
  t30 = dq1 * q1 * t5;

  // 'coriolis:37' t31 = dq2.*q2.*t4;
  t31 = dq2 * q2 * t4;

  // 'coriolis:38' t32 = dq3.*q3.*t3;
  t32 = dq3 * q3 * t3;

  // 'coriolis:39' t33 = dq4.*q4.*t2;
  t33 = dq4 * q4 * t2;

  // 'coriolis:40' t34 = Jw.*rk.*t2.*3.0;
  t34 = Jw * rk * t2 * 3.0F;

  // 'coriolis:41' t35 = Jw.*rk.*t3.*3.0;
  t35 = Jw * rk * t3 * 3.0F;

  // 'coriolis:42' t36 = q1.*q4;
  t36 = q1 * q4;

  // 'coriolis:43' t37 = Jw.*q1.*q2.*rk.*3.0;
  // 'coriolis:44' t38 = Jw.*q3.*q4.*rk.*3.0;
  // 'coriolis:45' t45 = COM_Y.*Mb.*t7.*2.0;
  // 'coriolis:46' t39 = t37+t38-t45;
  t39 = (Jw * q1 * q2 * rk * 3.0F + Jw * q3 * q4 * rk * 3.0F) - COM_Y * Mb * t7 *
    2.0F;

  // 'coriolis:47' t40 = COM_X.*Mb.*t7.*4.0;
  // 'coriolis:48' t41 = Jw.*q1.*q3.*rk.*6.0;
  // 'coriolis:49' t42 = t40+t41-Jw.*q2.*q4.*rk.*6.0;
  t42 = (COM_X * Mb * t7 * 4.0F + Jw * q1 * q3 * rk * 6.0F) - Jw * q2 * q4 * rk *
    6.0F;

  // 'coriolis:50' t43 = q2.*q3;
  // 'coriolis:51' t44 = t36+t43;
  t44 = t36 + q2 * q3;

  // 'coriolis:52' t46 = dq1.*rk.*t2.*4.0;
  t46 = dq1 * rk * t2 * 4.0F;

  // 'coriolis:53' t47 = dq1.*rk.*t3.*4.0;
  t47 = dq1 * rk * t3 * 4.0F;

  // 'coriolis:54' t48 = dq1.*rk.*t4.*4.0;
  t48 = dq1 * rk * t4 * 4.0F;

  // 'coriolis:55' t49 = dq1.*rk.*t5.*4.0;
  t49 = dq1 * rk * t5 * 4.0F;

  // 'coriolis:56' t50 = dq2.*rk.*t2.*4.0;
  t50 = dq2 * rk * t2 * 4.0F;

  // 'coriolis:57' t51 = dq2.*rk.*t3.*4.0;
  t51 = dq2 * rk * t3 * 4.0F;

  // 'coriolis:58' t52 = dq2.*rk.*t4.*4.0;
  t52 = dq2 * rk * t4 * 4.0F;

  // 'coriolis:59' t53 = dq2.*rk.*t5.*4.0;
  t53 = dq2 * rk * t5 * 4.0F;

  // 'coriolis:60' t54 = dq4.*rk.*t2.*4.0;
  t54 = dq4 * rk * t2 * 4.0F;

  // 'coriolis:61' t55 = dq4.*rk.*t3.*4.0;
  t55 = dq4 * rk * t3 * 4.0F;

  // 'coriolis:62' t56 = dq4.*rk.*t4.*4.0;
  t56 = dq4 * rk * t4 * 4.0F;

  // 'coriolis:63' t57 = dq4.*rk.*t5.*4.0;
  t57 = dq4 * rk * t5 * 4.0F;

  // 'coriolis:64' t58 = COM_Z.^2;
  t58 = COM_Z * COM_Z;

  // 'coriolis:65' t59 = rk.^2;
  t7 = rk * rk;

  // 'coriolis:66' t60 = COM_X.^2;
  t60 = COM_X * COM_X;

  // 'coriolis:67' t61 = COM_Y.^2;
  t61 = COM_Y * COM_Y;

  // 'coriolis:68' t62 = Jbx.*q2.*8.0;
  t62 = Jbx * q2 * 8.0F;

  // 'coriolis:69' t63 = Mb.*q2.*t58.*8.0;
  t63 = Mb * q2 * t58 * 8.0F;

  // 'coriolis:70' t64 = Jw.*q2.*t6.*t59.*6.0;
  t64 = Jw * q2 * t6 * t7 * 6.0F;

  // 'coriolis:71' t65 = Jby.*q3.*8.0;
  t65 = Jby * q3 * 8.0F;

  // 'coriolis:72' t66 = Mb.*q3.*t58.*8.0;
  t66 = Mb * q3 * t58 * 8.0F;

  // 'coriolis:73' t67 = Jw.*q3.*t6.*t59.*6.0;
  t67 = Jw * q3 * t6 * t7 * 6.0F;

  // 'coriolis:74' t68 = Jbz.*q4.*8.0;
  t68 = Jbz * q4 * 8.0F;

  // 'coriolis:75' t69 = Jw.*q4.*t6.*t59.*1.2e1;
  t69 = Jw * q4 * t6 * t7 * 12.0F;

  // 'coriolis:76' t70 = Mb.*q4.*t60.*8.0;
  t70 = Mb * q4 * t60 * 8.0F;

  // 'coriolis:77' t71 = COM_X.*COM_Y.*Mb.*q3.*8.0;
  t71 = COM_X * COM_Y * Mb * q3 * 8.0F;

  // 'coriolis:78' t72 = Mb.*q1.*t61.*8.0;
  t72 = Mb * q1 * t61 * 8.0F;

  // 'coriolis:79' t73 = COM_X.*COM_Z.*Mb.*q3.*8.0;
  t73 = COM_X * COM_Z * Mb * q3 * 8.0F;

  // 'coriolis:80' t74 = Mb.*q3.*t60.*8.0;
  t74 = Mb * q3 * t60 * 8.0F;

  // 'coriolis:81' t75 = Mb.*q3.*t61.*8.0;
  t75 = Mb * q3 * t61 * 8.0F;

  // 'coriolis:82' t76 = Mb.*q4.*t61.*8.0;
  t76 = Mb * q4 * t61 * 8.0F;

  // 'coriolis:83' t77 = COM_X.*COM_Y.*Mb.*q1.*8.0;
  t77 = COM_X * COM_Y * Mb * q1 * 8.0F;

  // 'coriolis:84' t78 = Jw.*q4.*t6.*t59.*6.0;
  t78 = Jw * q4 * t6 * t7 * 6.0F;

  // 'coriolis:85' t79 = COM_X.*COM_Y.*Mb.*q2.*8.0;
  t79 = COM_X * COM_Y * Mb * q2 * 8.0F;

  // 'coriolis:86' t80 = COM_Y.*COM_Z.*Mb.*q4.*8.0;
  t80 = COM_Y * COM_Z * Mb * q4 * 8.0F;

  // 'coriolis:87' t81 = Mb.*q1.*t60.*8.0;
  t81 = Mb * q1 * t60 * 8.0F;

  // 'coriolis:88' t82 = Mb.*q1.*t58.*8.0;
  t82 = Mb * q1 * t58 * 8.0F;

  // 'coriolis:89' t83 = COM_X.*COM_Y.*Mb.*q4.*8.0;
  t83 = COM_X * COM_Y * Mb * q4 * 8.0F;

  // 'coriolis:90' t84 = Jw.*q1.*t6.*t59.*6.0;
  t84 = Jw * q1 * t6 * t7 * 6.0F;

  // 'coriolis:91' t85 = Mb.*q2.*t60.*8.0;
  t85 = Mb * q2 * t60 * 8.0F;

  // 'coriolis:92' t86 = Mb.*q2.*t61.*8.0;
  t86 = Mb * q2 * t61 * 8.0F;

  // 'coriolis:93' t87 = COM_Y.*COM_Z.*Mb.*q1.*8.0;
  t87 = COM_Y * COM_Z * Mb * q1 * 8.0F;

  // 'coriolis:94' t88 = Jbx.*q1.*8.0;
  t88 = Jbx * q1 * 8.0F;

  // 'coriolis:95' t89 = Jbz.*q1.*8.0;
  t89 = Jbz * q1 * 8.0F;

  // 'coriolis:96' t90 = COM_Y.*COM_Z.*Mb.*q2.*8.0;
  t90 = COM_Y * COM_Z * Mb * q2 * 8.0F;

  // 'coriolis:97' t91 = Jbz.*q3.*8.0;
  t91 = Jbz * q3 * 8.0F;

  // 'coriolis:98' t92 = Mb.*q3.*t60.*4.0;
  t92 = Mb * q3 * t60 * 4.0F;

  // 'coriolis:99' t93 = Mb.*q3.*t58.*4.0;
  t93 = Mb * q3 * t58 * 4.0F;

  // 'coriolis:100' t94 = COM_X.*COM_Z.*Mb.*q1.*8.0;
  t94 = COM_X * COM_Z * Mb * q1 * 8.0F;

  // 'coriolis:101' t95 = Jbz.*q2.*8.0;
  t95 = Jbz * q2 * 8.0F;

  // 'coriolis:102' t96 = COM_X.*COM_Z.*Mb.*q4.*8.0;
  t96 = COM_X * COM_Z * Mb * q4 * 8.0F;

  // 'coriolis:103' t97 = Jbx.*q3.*8.0;
  t97 = Jbx * q3 * 8.0F;

  // 'coriolis:104' t98 = Jby.*q4.*8.0;
  t98 = Jby * q4 * 8.0F;

  // 'coriolis:105' t99 = COM_X.*COM_Z.*Mb.*q2.*8.0;
  t99 = COM_X * COM_Z * Mb * q2 * 8.0F;

  // 'coriolis:106' t100 = Mb.*q4.*t60.*4.0;
  t100 = Mb * q4 * t60 * 4.0F;

  // 'coriolis:107' t101 = Mb.*q4.*t61.*4.0;
  t101 = Mb * q4 * t61 * 4.0F;

  // 'coriolis:108' t102 = COM_Y.*COM_Z.*Mb.*q3.*8.0;
  t102 = COM_Y * COM_Z * Mb * q3 * 8.0F;

  // 'coriolis:109' t103 = Jby.*q2.*8.0;
  t103 = Jby * q2 * 8.0F;

  // 'coriolis:110' t104 = Jby.*q1.*8.0;
  t104 = Jby * q1 * 8.0F;

  // 'coriolis:111' t105 = Mb.*q4.*t58.*8.0;
  t105 = Mb * q4 * t58 * 8.0F;

  // 'coriolis:112' t106 = Jw.*q3.*t6.*t59.*1.2e1;
  t106 = Jw * q3 * t6 * t7 * 12.0F;

  // 'coriolis:113' t107 = t58+t60+t61;
  t107 = (t58 + t60) + t61;

  // 'coriolis:114' t108 = Mb.*q1.*t61.*4.0;
  t108 = Mb * q1 * t61 * 4.0F;

  // 'coriolis:115' t109 = Mb.*q1.*t58.*4.0;
  t109 = Mb * q1 * t58 * 4.0F;

  // 'coriolis:116' t110 = Mb.*q2.*t61.*4.0;
  t110 = Mb * q2 * t61 * 4.0F;

  // 'coriolis:117' t111 = Mb.*q2.*t58.*4.0;
  t111 = Mb * q2 * t58 * 4.0F;

  // 'coriolis:118' t112 = Jw.*q2.*t6.*t59.*1.2e1;
  t112 = Jw * q2 * t6 * t7 * 12.0F;

  // 'coriolis:119' t113 = Mb.*q4.*t58.*4.0;
  t113 = Mb * q4 * t58 * 4.0F;

  // 'coriolis:120' t114 = Jbx.*q4.*8.0;
  t114 = Jbx * q4 * 8.0F;

  // 'coriolis:121' t115 = Mb.*q3.*t61.*4.0;
  t115 = Mb * q3 * t61 * 4.0F;

  // 'coriolis:122' t116 = Mb.*q2.*t60.*4.0;
  t116 = Mb * q2 * t60 * 4.0F;

  // 'coriolis:123' t117 = Mb.*q1.*t60.*4.0;
  t117 = Mb * q1 * t60 * 4.0F;

  // 'coriolis:124' t118 = Jw.*q1.*t6.*t59.*1.2e1;
  t118 = Jw * q1 * t6 * t7 * 12.0F;

  // 'coriolis:125' t119 = dx.*q3.*t4;
  // 'coriolis:126' t120 = dx.*q3.*t2;
  // 'coriolis:127' t121 = dx.*q3.*t3;
  // 'coriolis:128' t122 = dx.*q3.*t5.*3.0;
  // 'coriolis:129' t123 = dy.*q2.*t2.*2.0;
  // 'coriolis:130' t124 = dq2.*q1.*q2.*rk.*8.0;
  // 'coriolis:131' t228 = dy.*q2.*t5.*2.0;
  // 'coriolis:132' t229 = dx.*q1.*q2.*q4.*2.0;
  // 'coriolis:133' t230 = dy.*q1.*q3.*q4.*4.0;
  // 'coriolis:134' t231 = dq2.*q3.*q4.*rk.*8.0;
  // 'coriolis:135' t125 = t46+t47+t48+t49+t119+t120+t121+t122+t123+t124-t228-t229-t230-t231; 
  t125 = ((((((((((((t46 + t47) + t48) + t49) + dx * q3 * t4) + dx * q3 * t2) +
                dx * q3 * t3) + dx * q3 * t5 * 3.0F) + dy * q2 * t2 * 2.0F) +
             dq2 * q1 * q2 * rk * 8.0F) - dy * q2 * t5 * 2.0F) - dx * q1 * q2 *
           q4 * 2.0F) - dy * q1 * q3 * q4 * 4.0F) - dq2 * q3 * q4 * rk * 8.0F;

  // 'coriolis:136' t126 = dy.*q1.*t4.*2.0;
  // 'coriolis:137' t127 = dx.*q1.*q2.*q3.*2.0;
  // 'coriolis:138' t128 = dy.*q2.*q3.*q4.*4.0;
  // 'coriolis:139' t129 = dq1.*q1.*q2.*rk.*8.0;
  // 'coriolis:140' t232 = dx.*q4.*t5;
  // 'coriolis:141' t233 = dx.*q4.*t2;
  // 'coriolis:142' t234 = dx.*q4.*t3;
  // 'coriolis:143' t235 = dx.*q4.*t4.*3.0;
  // 'coriolis:144' t236 = dy.*q1.*t3.*2.0;
  // 'coriolis:145' t237 = dq1.*q3.*q4.*rk.*8.0;
  // 'coriolis:146' t130 = t50+t51+t52+t53+t126+t127+t128+t129-t232-t233-t234-t235-t236-t237; 
  t130 = ((((((((((((t50 + t51) + t52) + t53) + dy * q1 * t4 * 2.0F) + dx * q1 *
                 q2 * q3 * 2.0F) + dy * q2 * q3 * q4 * 4.0F) + dq1 * q1 * q2 *
               rk * 8.0F) - dx * q4 * t5) - dx * q4 * t2) - dx * q4 * t3) - dx *
           q4 * t4 * 3.0F) - dy * q1 * t3 * 2.0F) - dq1 * q3 * q4 * rk * 8.0F;

  // 'coriolis:147' t131 = dx.*q1.*t2;
  t131 = dx * q1 * t2;

  // 'coriolis:148' t132 = dx.*q1.*t3.*3.0;
  t132 = dx * q1 * t3 * 3.0F;

  // 'coriolis:149' t133 = dx.*q1.*t4;
  t133 = dx * q1 * t4;

  // 'coriolis:150' t134 = dx.*q1.*t5;
  t134 = dx * q1 * t5;

  // 'coriolis:151' t135 = dy.*q4.*t3.*2.0;
  t135 = dy * q4 * t3 * 2.0F;

  // 'coriolis:152' t136 = dq3.*rk.*t2.*4.0;
  t7 = dq3 * rk * t2 * 4.0F;

  // 'coriolis:153' t137 = dq3.*rk.*t3.*4.0;
  t137 = dq3 * rk * t3 * 4.0F;

  // 'coriolis:154' t138 = dq3.*rk.*t4.*4.0;
  t138 = dq3 * rk * t4 * 4.0F;

  // 'coriolis:155' t139 = dq3.*rk.*t5.*4.0;
  t139 = dq3 * rk * t5 * 4.0F;

  // 'coriolis:156' t140 = dy.*q1.*q2.*q3.*4.0;
  t140 = dy * q1 * q2 * q3 * 4.0F;

  // 'coriolis:157' t141 = dq4.*q1.*q2.*rk.*8.0;
  t141 = dq4 * q1 * q2 * rk * 8.0F;

  // 'coriolis:158' t142 = dx.*q2.*t3;
  // 'coriolis:159' t143 = dx.*q2.*t2.*3.0;
  // 'coriolis:160' t144 = dx.*q2.*t4;
  // 'coriolis:161' t145 = dx.*q2.*t5;
  // 'coriolis:162' t146 = dy.*q3.*t2.*2.0;
  // 'coriolis:163' t147 = dy.*q1.*q2.*q4.*4.0;
  // 'coriolis:164' t148 = dq3.*q3.*q4.*rk.*8.0;
  // 'coriolis:165' t239 = dy.*q3.*t5.*2.0;
  // 'coriolis:166' t240 = dx.*q1.*q3.*q4.*2.0;
  // 'coriolis:167' t241 = dq3.*q1.*q2.*rk.*8.0;
  // 'coriolis:168' t149 = t54+t55+t56+t57+t142+t143+t144+t145+t146+t147+t148-t239-t240-t241; 
  t149 = ((((((((((((t54 + t55) + t56) + t57) + dx * q2 * t3) + dx * q2 * t2 *
                 3.0F) + dx * q2 * t4) + dx * q2 * t5) + dy * q3 * t2 * 2.0F) +
             dy * q1 * q2 * q4 * 4.0F) + dq3 * q3 * q4 * rk * 8.0F) - dy * q3 *
           t5 * 2.0F) - dx * q1 * q3 * q4 * 2.0F) - dq3 * q1 * q2 * rk * 8.0F;

  // 'coriolis:169' t150 = dy.*q1.*t2;
  // 'coriolis:170' t151 = dy.*q1.*t3;
  // 'coriolis:171' t152 = dy.*q1.*t4.*3.0;
  // 'coriolis:172' t153 = dy.*q1.*t5;
  // 'coriolis:173' t154 = dy.*q2.*q3.*q4.*2.0;
  // 'coriolis:174' t155 = dq4.*q1.*q3.*rk.*8.0;
  // 'coriolis:175' t156 = dq4.*q2.*q4.*rk.*8.0;
  // 'coriolis:176' t157 = t50+t51+t52+t53+t150+t151+t152+t153+t154+t155+t156;
  t157 = (((((((((t50 + t51) + t52) + t53) + dy * q1 * t2) + dy * q1 * t3) + dy *
             q1 * t4 * 3.0F) + dy * q1 * t5) + dy * q2 * q3 * q4 * 2.0F) + dq4 *
          q1 * q3 * rk * 8.0F) + dq4 * q2 * q4 * rk * 8.0F;

  // 'coriolis:177' t158 = dq3.*q1.*q3.*rk.*8.0;
  // 'coriolis:178' t159 = dq3.*q2.*q4.*rk.*8.0;
  // 'coriolis:179' t242 = dy.*q2.*t3;
  // 'coriolis:180' t243 = dy.*q2.*t2;
  // 'coriolis:181' t244 = dy.*q2.*t4;
  // 'coriolis:182' t245 = dy.*q2.*t5.*3.0;
  // 'coriolis:183' t246 = dy.*q1.*q3.*q4.*2.0;
  // 'coriolis:184' t160 = t46+t47+t48+t49+t158+t159-t242-t243-t244-t245-t246;
  t160 = (((((((((t46 + t47) + t48) + t49) + dq3 * q1 * q3 * rk * 8.0F) + dq3 *
              q2 * q4 * rk * 8.0F) - dy * q2 * t3) - dy * q2 * t2) - dy * q2 *
           t4) - dy * q2 * t5 * 3.0F) - dy * q1 * q3 * q4 * 2.0F;

  // 'coriolis:185' t161 = dy.*q3.*t4;
  // 'coriolis:186' t162 = dy.*q3.*t2.*3.0;
  // 'coriolis:187' t163 = dy.*q3.*t3;
  // 'coriolis:188' t164 = dy.*q3.*t5;
  // 'coriolis:189' t165 = dy.*q1.*q2.*q4.*2.0;
  // 'coriolis:190' t166 = dq2.*q1.*q3.*rk.*8.0;
  // 'coriolis:191' t167 = dq2.*q2.*q4.*rk.*8.0;
  // 'coriolis:192' t168 = t54+t55+t56+t57+t161+t162+t163+t164+t165+t166+t167;
  t168 = (((((((((t54 + t55) + t56) + t57) + dy * q3 * t4) + dy * q3 * t2 * 3.0F)
             + dy * q3 * t3) + dy * q3 * t5) + dy * q1 * q2 * q4 * 2.0F) + dq2 *
          q1 * q3 * rk * 8.0F) + dq2 * q2 * q4 * rk * 8.0F;

  // 'coriolis:193' t169 = dq1.*q1.*q3.*rk.*8.0;
  // 'coriolis:194' t170 = dq1.*q2.*q4.*rk.*8.0;
  // 'coriolis:195' t247 = dy.*q4.*t5;
  // 'coriolis:196' t248 = dy.*q4.*t2;
  // 'coriolis:197' t249 = dy.*q4.*t3.*3.0;
  // 'coriolis:198' t250 = dy.*q4.*t4;
  // 'coriolis:199' t251 = dy.*q1.*q2.*q3.*2.0;
  // 'coriolis:200' t171 = t136+t137+t138+t139+t169+t170-t247-t248-t249-t250-t251; 
  t171 = (((((((((t7 + t137) + t138) + t139) + dq1 * q1 * q3 * rk * 8.0F) + dq1 *
              q2 * q4 * rk * 8.0F) - dy * q4 * t5) - dy * q4 * t2) - dy * q4 *
           t3 * 3.0F) - dy * q4 * t4) - dy * q1 * q2 * q3 * 2.0F;

  // 'coriolis:201' t172 = Mb.*dq1.*q1.*t107.*4.0;
  t172 = Mb * dq1 * q1 * t107 * 4.0F;

  // 'coriolis:202' t173 = COM_X.*COM_Z.*Mb.*q4.*1.6e1;
  // 'coriolis:203' t174 = -t62-t63+t64+t71+t85+t87+t95+t173;
  t174 = ((((((-t62 - t63) + t64) + t71) + t85) + t87) + t95) + COM_X * COM_Z *
    Mb * q4 * 16.0F;

  // 'coriolis:204' t175 = COM_Y.*COM_Z.*Mb.*q4.*1.6e1;
  t175 = COM_Y * COM_Z * Mb * q4 * 16.0F;

  // 'coriolis:205' t176 = t72-t73+t82+t83+t84+t88;
  t176 = ((((t72 - t73) + t82) + t83) + t84) + t88;

  // 'coriolis:206' t177 = COM_X.*COM_Y.*Mb.*q2.*1.6e1;
  // 'coriolis:207' t178 = -t65-t74+t75+t80+t94+t97+t177;
  // 'coriolis:208' t179 = dq4.*t178;
  t179 = dq4 * ((((((-t65 - t74) + t75) + t80) + t94) + t97) + COM_X * COM_Y *
                Mb * q2 * 16.0F);

  // 'coriolis:209' t180 = COM_Y.*COM_Z.*Mb.*q3.*1.6e1;
  // 'coriolis:210' t181 = -t68-t76+t77-t78+t98+t99+t105+t180;
  // 'coriolis:211' t182 = dq2.*t181;
  t182 = dq2 * (((((((-t68 - t76) + t77) - t78) + t98) + t99) + t105) + COM_Y *
                COM_Z * Mb * q3 * 16.0F);

  // 'coriolis:212' t183 = t81+t82-t83+t84+t90+t104;
  t183 = ((((t81 + t82) - t83) + t84) + t90) + t104;

  // 'coriolis:213' t184 = COM_X.*COM_Y.*Mb.*q3.*1.6e1;
  // 'coriolis:214' t185 = -t62+t85-t86-t87+t96+t103+t184;
  t185 = (((((-t62 + t85) - t86) - t87) + t96) + t103) + COM_X * COM_Y * Mb * q3
    * 16.0F;

  // 'coriolis:215' t186 = COM_X.*COM_Z.*Mb.*q3.*1.6e1;
  // 'coriolis:216' t187 = t81-t82-t83+t84-t88+t89-t90+t186;
  // 'coriolis:217' t188 = dq4.*t187;
  t188 = dq4 * (((((((t81 - t82) - t83) + t84) - t88) + t89) - t90) + COM_X *
                COM_Z * Mb * q3 * 16.0F);

  // 'coriolis:218' t189 = t80+t91+t92-t93-t94+t106+t115;
  // 'coriolis:219' t190 = dq2.*t189;
  t190 = dq2 * ((((((t80 + t91) + t92) - t93) - t94) + t106) + t115);

  // 'coriolis:220' t191 = t85+t86+t87+t95+t96+t112;
  // 'coriolis:221' t273 = dq3.*t191;
  t273 = dq3 * (((((t85 + t86) + t87) + t95) + t96) + t112);

  // 'coriolis:222' t192 = t188+t190-t273;
  t192 = (t188 + t190) - t273;

  // 'coriolis:223' t193 = COM_X.*COM_Z.*Mb.*q1.*1.6e1;
  // 'coriolis:224' t194 = t66-t67-t74+t79-t80-t91+t97+t193;
  // 'coriolis:225' t195 = dq4.*t194;
  t195 = dq4 * (((((((t66 - t67) - t74) + t79) - t80) - t91) + t97) + COM_X *
                COM_Z * Mb * q1 * 16.0F);

  // 'coriolis:226' t196 = COM_X.*COM_Y.*Mb.*q1.*1.6e1;
  t196 = COM_X * COM_Y * Mb * q1 * 16.0F;

  // 'coriolis:227' t197 = t77+t78+t98+t100-t101+t102+t113;
  // 'coriolis:228' t198 = t63+t64+t71+t85-t87+t103;
  // 'coriolis:229' t199 = dq4.*t198;
  // 'coriolis:230' t279 = COM_X.*COM_Y.*Mb.*q4.*1.6e1;
  // 'coriolis:231' t200 = -t72+t73+t81-t88+t90+t104-t279;
  // 'coriolis:232' t201 = dq3.*t200;
  // 'coriolis:233' t278 = dq2.*t197;
  // 'coriolis:234' t202 = t199+t201-t278;
  t202 = (dq4 * (((((t63 + t64) + t71) + t85) - t87) + t103) + dq3 * ((((((-t72
    + t73) + t81) - t88) + t90) + t104) - COM_X * COM_Y * Mb * q4 * 16.0F)) -
    dq2 * ((((((t77 + t78) + t98) + t100) - t101) + t102) + t113);

  // 'coriolis:235' t203 = t70+t77+t78+t98+t102+t105;
  // 'coriolis:236' t204 = dq4.*t203;
  // 'coriolis:237' t205 = t74+t75+t80+t91-t94+t106;
  // 'coriolis:238' t206 = dq3.*t205;
  // 'coriolis:239' t207 = Mb.*dq2.*q2.*t107.*4.0;
  // 'coriolis:240' t208 = t204+t206+t207;
  t57 = (dq4 * (((((t70 + t77) + t78) + t98) + t102) + t105) + dq3 * (((((t74 +
              t75) + t80) + t91) - t94) + t106)) + Mb * dq2 * q2 * t107 * 4.0F;

  // 'coriolis:241' t209 = COM_Y.*COM_Z.*Mb.*q1.*1.6e1;
  // 'coriolis:242' t210 = -t63+t64-t71+t86+t95+t96-t103+t209;
  t85 = ((((((-t63 + t64) - t71) + t86) + t95) + t96) - t103) + COM_Y * COM_Z *
    Mb * q1 * 16.0F;

  // 'coriolis:243' t211 = -t83+t84+t90+t104-t108+t109+t117;
  // 'coriolis:244' t212 = dq3.*t211;
  t106 = dq3 * ((((((-t83 + t84) + t90) + t104) - t108) + t109) + t117);

  // 'coriolis:245' t280 = COM_Y.*COM_Z.*Mb.*q2.*1.6e1;
  // 'coriolis:246' t213 = t72+t73-t82+t83+t84+t89-t104-t280;
  // 'coriolis:247' t214 = dq4.*t213;
  // 'coriolis:248' t215 = t87+t95+t96+t110-t111+t112+t116;
  // 'coriolis:249' t281 = dq3.*t215;
  // 'coriolis:250' t216 = t214-t281;
  t54 = dq4 * (((((((t72 + t73) - t82) + t83) + t84) + t89) - t104) - COM_Y *
               COM_Z * Mb * q2 * 16.0F) - dq3 * ((((((t87 + t95) + t96) + t110)
    - t111) + t112) + t116);

  // 'coriolis:251' t217 = -t77+t78+t99-t100+t101+t113+t114;
  // 'coriolis:252' t218 = dq3.*t217;
  // 'coriolis:253' t219 = t66+t67+t75+t79+t94+t97;
  // 'coriolis:254' t282 = dq4.*t219;
  // 'coriolis:255' t220 = t218-t282;
  t55 = dq3 * ((((((-t77 + t78) + t99) - t100) + t101) + t113) + t114) - dq4 *
    (((((t66 + t67) + t75) + t79) + t94) + t97);

  // 'coriolis:256' t221 = t76-t77+t78+t99+t105+t114;
  // 'coriolis:257' t222 = dq4.*t221;
  // 'coriolis:258' t223 = Mb.*dq3.*q3.*t107.*4.0;
  // 'coriolis:259' t224 = t222+t223;
  t56 = dq4 * (((((t76 - t77) + t78) + t99) + t105) + t114) + Mb * dq3 * q3 *
    t107 * 4.0F;

  // 'coriolis:260' t225 = t67+t79-t92+t93+t94+t97+t115;
  t49 = (((((t67 + t79) - t92) + t93) + t94) + t97) + t115;

  // 'coriolis:261' t226 = t64+t71-t87+t103-t110+t111+t116;
  t47 = (((((t64 + t71) - t87) + t103) - t110) + t111) + t116;

  // 'coriolis:262' t227 = t73+t89-t90+t108-t109+t117+t118;
  t48 = (((((t73 + t89) - t90) + t108) - t109) + t117) + t118;

  // 'coriolis:263' t238 = t131+t132+t133+t134+t135-t136-t137-t138-t139+t140+t141-dy.*q4.*t4.*2.0-dx.*q2.*q3.*q4.*2.0-dq4.*q3.*q4.*rk.*8.0; 
  t53 = ((((((((((((t131 + t132) + t133) + t134) + t135) - t7) - t137) - t138) -
             t139) + t140) + t141) - dy * q4 * t4 * 2.0F) - dx * q2 * q3 * q4 *
         2.0F) - dq4 * q3 * q4 * rk * 8.0F;

  // 'coriolis:264' t252 = t62+t63+t64-t71+t86-t96;
  // 'coriolis:265' t253 = dq2.*t252;
  // 'coriolis:266' t254 = t65+t66+t67+t74-t79-t80;
  // 'coriolis:267' t255 = dq3.*t254;
  // 'coriolis:268' t256 = t68+t69+t70+t76-t99-t102;
  // 'coriolis:269' t257 = dq4.*t256;
  // 'coriolis:270' t258 = t172+t253+t255+t257;
  t46 = ((t172 + dq2 * (((((t62 + t63) + t64) - t71) + t86) - t96)) + dq3 *
         (((((t65 + t66) + t67) + t74) - t79) - t80)) + dq4 * (((((t68 + t69) +
    t70) + t76) - t99) - t102);

  // 'coriolis:271' t259 = dq3.*t174;
  // 'coriolis:272' t260 = -t65-t66+t67+t75+t79+t91-t94+t175;
  // 'coriolis:273' t261 = t68+t69-t99+t100+t101-t102-t113;
  // 'coriolis:274' t262 = dq1.*t261;
  // 'coriolis:275' t263 = t72+t73+t81+t89-t90+t118;
  // 'coriolis:276' t264 = t259+t262-dq2.*t260-dq4.*t263;
  t52 = ((dq3 * t174 + dq1 * ((((((t68 + t69) - t99) + t100) + t101) - t102) -
           t113)) - dq2 * (((((((-t65 - t66) + t67) + t75) + t79) + t91) - t94)
          + t175)) - dq4 * (((((t72 + t73) + t81) + t89) - t90) + t118);

  // 'coriolis:277' t265 = t68+t70+t77+t78-t102-t105-t114-COM_X.*COM_Z.*Mb.*q2.*1.6e1; 
  // 'coriolis:278' t266 = dq3.*t265;
  // 'coriolis:279' t267 = t62+t64-t71-t96+t110+t111-t116;
  // 'coriolis:280' t268 = dq1.*t267;
  // 'coriolis:281' t269 = t179+t266+t268-dq2.*t176;
  t51 = ((t179 + dq3 * (((((((t68 + t70) + t77) + t78) - t102) - t105) - t114) -
                        COM_X * COM_Z * Mb * q2 * 16.0F)) + dq1 * ((((((t62 +
    t64) - t71) - t96) + t110) + t111) - t116)) - dq2 * t176;

  // 'coriolis:282' t270 = t65+t67-t79-t80+t92+t93-t115;
  // 'coriolis:283' t271 = dq1.*t270;
  // 'coriolis:284' t272 = t182+t271-dq3.*t183-dq4.*t185;
  t50 = ((t182 + dq1 * ((((((t65 + t67) - t79) - t80) + t92) + t93) - t115)) -
         dq3 * t183) - dq4 * t185;

  // 'coriolis:285' t274 = -t73+t83+t84+t88+t108+t109-t117;
  // 'coriolis:286' t275 = t70-t76+t98-t99+t102-t114+t196;
  // 'coriolis:287' t276 = dq3.*t275;
  // 'coriolis:288' t277 = t195+t276-dq2.*t274;
  t7 = (t195 + dq3 * ((((((t70 - t76) + t98) - t99) + t102) - t114) + t196)) -
    dq2 * ((((((-t73 + t83) + t84) + t88) + t108) + t109) - t117);

  // 'coriolis:289' C = reshape([Jw.*t6.*(t26+t27+t28+t29+t30+t31+t32+t33+dq1.*q1.*t3.*3.0+dq1.*q1.*t4+dq2.*q2.*t2.*3.0+dq3.*q3.*t2+dq2.*q2.*t5+dq3.*q3.*t5.*3.0+dq4.*q4.*t3+dq4.*q4.*t4.*3.0-dq1.*q2.*q3.*q4.*2.0-dq2.*q1.*q3.*q4.*2.0-dq3.*q1.*q2.*q4.*2.0-dq4.*q1.*q2.*q3.*2.0).*3.0,t25,Jw.*q4.*t6.*t125.*(-3.0./2.0)-Jw.*q3.*t6.*t130.*(3.0./2.0)-Jw.*q1.*t6.*t149.*(3.0./2.0)+Jw.*q2.*t6.*(t131+t132+t133+t134+t135+t140+t141-dy.*q4.*t4.*2.0-dq3.*rk.*t2.*4.0-dq3.*rk.*t3.*4.0-dq3.*rk.*t4.*4.0-dq3.*rk.*t5.*4.0-dx.*q2.*q3.*q4.*2.0-dq4.*q3.*q4.*rk.*8.0).*(3.0./2.0),Jw.*q1.*t6.*t125.*(-3.0./2.0)+Jw.*q2.*t6.*t130.*(3.0./2.0)+Jw.*q4.*t6.*t149.*(3.0./2.0)+Jw.*q3.*t6.*t238.*(3.0./2.0),Jw.*q2.*t6.*t125.*(3.0./2.0)+Jw.*q1.*t6.*t130.*(3.0./2.0)-Jw.*q3.*t6.*t149.*(3.0./2.0)+Jw.*q4.*t6.*t238.*(3.0./2.0),0.0,t25,Jw.*t6.*(t26+t27+t28+t29+t30+t31+t32+t33+dq1.*q1.*t3+dq1.*q1.*t4.*3.0+dq2.*q2.*t2+dq3.*q3.*t2.*3.0+dq2.*q2.*t5.*3.0+dq3.*q3.*t5+dq4.*q4.*t3.*3.0+dq4.*q4.*t4+dq1.*q2.*q3.*q4.*2.0+dq2.*q1.*q3.*q4.*2.0+dq3.*q1.*q2.*q4.*2.0+dq4.*q1.*q2.*q3.*2.0).*3.0,Jw.*q2.*t6.*t157.*(3.0./2.0)+Jw.*q1.*t6.*t160.*(3.0./2.0)-Jw.*q4.*t6.*t168.*(3.0./2.0)-Jw.*q3.*t6.*t171.*(3.0./2.0),Jw.*q3.*t6.*t157.*(3.0./2.0)-Jw.*q4.*t6.*t160.*(3.0./2.0)-Jw.*q1.*t6.*t168.*(3.0./2.0)+Jw.*q2.*t6.*t171.*(3.0./2.0),Jw.*q4.*t6.*t157.*(3.0./2.0)+Jw.*q3.*t6.*t160.*(3.0./2.0)+Jw.*q2.*t6.*t168.*(3.0./2.0)+Jw.*q1.*t6.*t171.*(3.0./2.0),0.0,dq3.*t6.*(t8+t9+t10-Jw.*rk.*t2.*3.0-Jw.*rk.*t3.*3.0)+dq1.*t6.*t13+dq4.*t6.*t16-Jw.*dq2.*rk.*t6.*(t36-q2.*q3).*6.0,-dq2.*t6.*(t8-t9+t10-t34+t35)-dq1.*t6.*t39+dq4.*t6.*t42-Jw.*dq3.*rk.*t6.*t44.*6.0,-q4.*(t182+dq1.*(t65+t67-t79-t80+t92+t93-Mb.*q3.*t61.*4.0)-dq3.*t183-dq4.*t185)-q2.*(t172+dq3.*(t65+t66+t67+t74-COM_X.*COM_Y.*Mb.*q2.*8.0-COM_Y.*COM_Z.*Mb.*q4.*8.0)+dq2.*(t62+t63+t64+t86-COM_X.*COM_Y.*Mb.*q3.*8.0-COM_X.*COM_Z.*Mb.*q4.*8.0)+dq4.*(t68+t69+t70+t76-COM_X.*COM_Z.*Mb.*q2.*8.0-COM_Y.*COM_Z.*Mb.*q3.*8.0))-q1.*(t179-dq2.*t176+dq3.*(t68+t70+t77+t78-Jbx.*q4.*8.0-Mb.*q4.*t58.*8.0-COM_X.*COM_Z.*Mb.*q2.*1.6e1-COM_Y.*COM_Z.*Mb.*q3.*8.0)+dq1.*(t62+t64-t71+t110+t111-Mb.*q2.*t60.*4.0-COM_X.*COM_Z.*Mb.*q4.*8.0))-q3.*(dq4.*(t72+t73+t81+t89+t118-COM_Y.*COM_Z.*Mb.*q2.*8.0)-dq3.*t174+dq2.*(-t65-t66+t67+t75+t79+t91+t175-COM_X.*COM_Z.*Mb.*q1.*8.0)-dq1.*(t68+t69+t100+t101-Mb.*q4.*t58.*4.0-COM_X.*COM_Z.*Mb.*q2.*8.0-COM_Y.*COM_Z.*Mb.*q3.*8.0)),-q3.*t258-q2.*t264-q1.*t272+q4.*t269,-q4.*t258-q1.*t264-q3.*t269+q2.*t272,dq1+beta.*q1,dq4.*t6.*(t8-t9-t10+t34+t35)+dq2.*t6.*t13-dq3.*t6.*t16,dq2.*t6.*t39+dq3.*t6.*t42+Jw.*dq4.*rk.*t6.*t44.*6.0,-q2.*(t195+dq3.*(t70-t76+t98-t99+t102+t196-Jbx.*q4.*8.0)-dq2.*(-t73+t83+t84+t88+t108+t109-Mb.*q1.*t60.*4.0))-q4.*t192-q3.*t202+q1.*t208,-q1.*t192+q2.*t202-q4.*t208-q3.*t277,q1.*t202+q3.*t208-q4.*t277+q2.*(t188+t190-t273),dq2+beta.*q2,-dq4.*t6.*(Jw.*q1.*q4.*rk.*6.0-Jw.*q2.*q3.*rk.*6.0)-dq3.*t6.*t13,dq4.*t6.*(t8+t9-t10+t34-t35)-dq3.*t6.*t39,q1.*t216+q3.*t220+q4.*t224+q2.*(t212-dq4.*t210),-q4.*t216-q2.*t220+q1.*t224+q3.*(t212-dq4.*t210),q3.*t216-q1.*t220-q2.*t224+q4.*(t212-dq4.*t210),dq3+beta.*q3,-dq4.*t6.*t13,dq4.*t6.*t39,-dq4.*q1.*t226+dq4.*q2.*t227-dq4.*q4.*t225-Mb.*dq4.*q3.*q4.*t107.*4.0,-dq4.*q1.*t225+dq4.*q3.*t227+dq4.*q4.*t226+Mb.*dq4.*q2.*q4.*t107.*4.0,dq4.*q2.*t225-dq4.*q3.*t226+dq4.*q4.*t227+Mb.*dq4.*q1.*q4.*t107.*4.0,dq4+beta.*q4],[6,6]); 
  x[0] = Jw * t6 * (((((((((((((((((((t26 + t27) + t28) + t29) + t30) + t31) +
    t32) + t33) + dq1 * q1 * t3 * 3.0F) + dq1 * q1 * t4) + dq2 * q2 * t2 * 3.0F)
    + dq3 * q3 * t2) + dq2 * q2 * t5) + dq3 * q3 * t5 * 3.0F) + dq4 * q4 * t3) +
                        dq4 * q4 * t4 * 3.0F) - dq1 * q2 * q3 * q4 * 2.0F) - dq2
                      * q1 * q3 * q4 * 2.0F) - dq3 * q1 * q2 * q4 * 2.0F) - dq4 *
                    q1 * q2 * q3 * 2.0F) * 3.0F;
  x[1] = t25;
  x[2] = ((Jw * q4 * t6 * t125 * -1.5F - Jw * q3 * t6 * t130 * 1.5F) - Jw * q1 *
          t6 * t149 * 1.5F) + Jw * q2 * t6 * (((((((((((((t131 + t132) + t133) +
    t134) + t135) + t140) + t141) - dy * q4 * t4 * 2.0F) - dq3 * rk * t2 * 4.0F)
    - dq3 * rk * t3 * 4.0F) - dq3 * rk * t4 * 4.0F) - dq3 * rk * t5 * 4.0F) - dx
    * q2 * q3 * q4 * 2.0F) - dq4 * q3 * q4 * rk * 8.0F) * 1.5F;
  x[3] = ((Jw * q1 * t6 * t125 * -1.5F + Jw * q2 * t6 * t130 * 1.5F) + Jw * q4 *
          t6 * t149 * 1.5F) + Jw * q3 * t6 * t53 * 1.5F;
  x[4] = ((Jw * q2 * t6 * t125 * 1.5F + Jw * q1 * t6 * t130 * 1.5F) - Jw * q3 *
          t6 * t149 * 1.5F) + Jw * q4 * t6 * t53 * 1.5F;
  x[5] = 0.0F;
  x[6] = t25;
  x[7] = Jw * t6 * (((((((((((((((((((t26 + t27) + t28) + t29) + t30) + t31) +
    t32) + t33) + dq1 * q1 * t3) + dq1 * q1 * t4 * 3.0F) + dq2 * q2 * t2) + dq3 *
    q3 * t2 * 3.0F) + dq2 * q2 * t5 * 3.0F) + dq3 * q3 * t5) + dq4 * q4 * t3 *
    3.0F) + dq4 * q4 * t4) + dq1 * q2 * q3 * q4 * 2.0F) + dq2 * q1 * q3 * q4 *
                      2.0F) + dq3 * q1 * q2 * q4 * 2.0F) + dq4 * q1 * q2 * q3 *
                    2.0F) * 3.0F;
  x[8] = ((Jw * q2 * t6 * t157 * 1.5F + Jw * q1 * t6 * t160 * 1.5F) - Jw * q4 *
          t6 * t168 * 1.5F) - Jw * q3 * t6 * t171 * 1.5F;
  x[9] = ((Jw * q3 * t6 * t157 * 1.5F - Jw * q4 * t6 * t160 * 1.5F) - Jw * q1 *
          t6 * t168 * 1.5F) + Jw * q2 * t6 * t171 * 1.5F;
  x[10] = ((Jw * q4 * t6 * t157 * 1.5F + Jw * q3 * t6 * t160 * 1.5F) + Jw * q2 *
           t6 * t168 * 1.5F) + Jw * q1 * t6 * t171 * 1.5F;
  x[11] = 0.0F;
  x[12] = ((dq3 * t6 * ((((t8 + t9) + t10) - Jw * rk * t2 * 3.0F) - Jw * rk * t3
                        * 3.0F) + dq1 * t6 * t13) + dq4 * t6 * t16) - Jw * dq2 *
    rk * t6 * (t36 - q2 * q3) * 6.0F;
  x[13] = ((-dq2 * t6 * ((((t8 - t9) + t10) - t34) + t35) - dq1 * t6 * t39) +
           dq4 * t6 * t42) - Jw * dq3 * rk * t6 * t44 * 6.0F;
  x[14] = ((-q4 * (((t182 + dq1 * ((((((t65 + t67) - t79) - t80) + t92) + t93) -
    Mb * q3 * t61 * 4.0F)) - dq3 * t183) - dq4 * t185) - q2 * (((t172 + dq3 *
    (((((t65 + t66) + t67) + t74) - COM_X * COM_Y * Mb * q2 * 8.0F) - COM_Y *
     COM_Z * Mb * q4 * 8.0F)) + dq2 * (((((t62 + t63) + t64) + t86) - COM_X *
    COM_Y * Mb * q3 * 8.0F) - COM_X * COM_Z * Mb * q4 * 8.0F)) + dq4 * (((((t68
    + t69) + t70) + t76) - COM_X * COM_Z * Mb * q2 * 8.0F) - COM_Y * COM_Z * Mb *
              q3 * 8.0F))) - q1 * (((t179 - dq2 * t176) + dq3 * (((((((t68 + t70)
    + t77) + t78) - Jbx * q4 * 8.0F) - Mb * q4 * t58 * 8.0F) - COM_X * COM_Z *
    Mb * q2 * 16.0F) - COM_Y * COM_Z * Mb * q3 * 8.0F)) + dq1 * ((((((t62 + t64)
    - t71) + t110) + t111) - Mb * q2 * t60 * 4.0F) - COM_X * COM_Z * Mb * q4 *
             8.0F))) - q3 * (((dq4 * (((((t72 + t73) + t81) + t89) + t118) -
    COM_Y * COM_Z * Mb * q2 * 8.0F) - dq3 * t174) + dq2 * (((((((-t65 - t66) +
    t67) + t75) + t79) + t91) + t175) - COM_X * COM_Z * Mb * q1 * 8.0F)) - dq1 *
    ((((((t68 + t69) + t100) + t101) - Mb * q4 * t58 * 4.0F) - COM_X * COM_Z *
      Mb * q2 * 8.0F) - COM_Y * COM_Z * Mb * q3 * 8.0F));
  x[15] = ((-q3 * t46 - q2 * t52) - q1 * t50) + q4 * t51;
  x[16] = ((-q4 * t46 - q1 * t52) - q3 * t51) + q2 * t50;
  x[17] = dq1 + (float)beta * q1;
  x[18] = (dq4 * t6 * ((((t8 - t9) - t10) + t34) + t35) + dq2 * t6 * t13) - dq3 *
    t6 * t16;
  x[19] = (dq2 * t6 * t39 + dq3 * t6 * t42) + Jw * dq4 * rk * t6 * t44 * 6.0F;
  x[20] = ((-q2 * ((t195 + dq3 * ((((((t70 - t76) + t98) - t99) + t102) + t196)
    - Jbx * q4 * 8.0F)) - dq2 * ((((((-t73 + t83) + t84) + t88) + t108) + t109)
              - Mb * q1 * t60 * 4.0F)) - q4 * t192) - q3 * t202) + q1 * t57;
  x[21] = ((-q1 * t192 + q2 * t202) - q4 * t57) - q3 * t7;
  x[22] = ((q1 * t202 + q3 * t57) - q4 * t7) + q2 * ((t188 + t190) - t273);
  x[23] = dq2 + (float)beta * q2;
  x[24] = -dq4 * t6 * (Jw * q1 * q4 * rk * 6.0F - Jw * q2 * q3 * rk * 6.0F) -
    dq3 * t6 * t13;
  x[25] = dq4 * t6 * ((((t8 + t9) - t10) + t34) - t35) - dq3 * t6 * t39;
  x[26] = ((q1 * t54 + q3 * t55) + q4 * t56) + q2 * (t106 - dq4 * t85);
  x[27] = ((-q4 * t54 - q2 * t55) + q1 * t56) + q3 * (t106 - dq4 * t85);
  x[28] = ((q3 * t54 - q1 * t55) - q2 * t56) + q4 * (t106 - dq4 * t85);
  x[29] = dq3 + (float)beta * q3;
  x[30] = -dq4 * t6 * t13;
  x[31] = dq4 * t6 * t39;
  x[32] = ((-dq4 * q1 * t47 + dq4 * q2 * t48) - dq4 * q4 * t49) - Mb * dq4 * q3 *
    q4 * t107 * 4.0F;
  x[33] = ((-dq4 * q1 * t49 + dq4 * q3 * t48) + dq4 * q4 * t47) + Mb * dq4 * q2 *
    q4 * t107 * 4.0F;
  x[34] = ((dq4 * q2 * t49 - dq4 * q3 * t47) + dq4 * q4 * t48) + Mb * dq4 * q1 *
    q4 * t107 * 4.0F;
  x[35] = dq4 + (float)beta * q4;
  i4 = 0;
  i5 = 0;
  i6 = 0;
  for (i7 = 0; i7 < 36; i7++) {
    C[i5 + 6 * i4] = x[i6];
    i4++;
    if (i4 > 5) {
      i4 = 0;
      i5++;
    }

    i6++;
  }
}

//
// File trailer for coriolis.cpp
//
// [EOF]
//
