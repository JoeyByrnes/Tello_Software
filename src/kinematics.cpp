// #include <cmath>
// #include <math.h>

// // function p = fcn_ik_q_2_p(q)
// float p[6];
// float q[6]; // temporary

// // first actuation group 
// // hip yaw (q1) - hip yaw actuator (p1)
// p[1] = q[1];


// // second actuation group - hip differential
// // {hip roll (q2), hip pitch (q3)} - {actuator2 (p2), actuator3 (p3)}
// float a1 = 57.0/2500.0 - (8.0*cos(q[2])*sin(q[3]))/625.0 - (7.0*sin(q[2]))/625.0;
// float b1 = -(8.0*cos(q[3]))/625.0;
// float c1 = (49.0*cos(q[2]))/5000.0 + (399.0*sin(q[2]))/20000.0 + (57.0*cos(q[2])*sin(q[3]))/2500.0 - (7.0*sin(q[2])*sin(q[3]))/625.0 - 3021.0/160000.0;
// float d1 = sqrt(a1*a1 + b1*b1 - c1*c1);
// float a2 = (7.0*sin(q[2]))/625.0 - (8.0*cos(q[2])*sin(q[3]))/625.0 + 57.0/2500.0;
// float b2 = -(8.0*cos(q[3]))/625.0;
// float c2 = (49.0*cos(q[2]))/5000.0 - (399.0*sin(q[2]))/20000.0 + (57.0*cos(q[2])*sin(q[3]))/2500.0 + (7.0*sin(q[2])*sin(q[3]))/625.0 - 3021.0/160000.0;
// float d2 = sqrt(a2*a2 + b2*b2 - c2*c2);

// p[2] = atan2(-b1*d1 + a1*c1, b1*c1+a1*d1);
// p[3] = atan2(-b2*d2 + a2*c2, b2*c2+a2*d2);


// // third actuation group - knee-ankle differential
// // {knee pitch (q4), ankle pitch (q5)} - {actuator4 (p4), actuator5 (p5)}
// float alpha_0 = (180.0-59.16)*M_PI/180.0;  // alpha_0
// float beta_0 = (180.0-66)*M_PI/180.0;   // beta_0
// float beta_ = q[5] + beta_0;
// float a3 = - (21.0*sin(M_PI/9))/6250.0 - (7*sin(beta_))/2500.0;
// float b3 = 13/625 - (7*cos(beta_))/2500 - (21.0*cos(M_PI/9))/6250.0;
// float c3 = (273*cos(M_PI/9))/12500 + (91*cos(beta_))/5000 - (147*cos(M_PI/9)*cos(beta_))/50000.0 - (147*sin(M_PI/9)*sin(beta_))/50000.0 - 163349.0/6250000.0;
// float d3 = sqrt(a3*a3 + b3*b3 - c3*c3);
// float alpha_ = atan2( b3*d3 + a3*c3, b3*c3-a3*d3);

// p[4] = q[4] - (alpha_ - alpha_0);
// p[5] = q[4] + (alpha_ - alpha_0);