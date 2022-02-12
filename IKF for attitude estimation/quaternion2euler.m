function [euler] = quaternion2euler(quat) 
% function euler = quaternion2euler(q)

euler = zeros(3,1);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
euler(3) = atan2( 2 * q1 * q2 + 2 * q0 * q3 , 2 * q0 * q0 + 2 * q1 * q1 - 1);
euler(2) = asin(-2*q1*q3 + 2*q0*q2);
euler(1) = atan2( 2 * q2 * q3 + 2 * q0 * q1 , 2 * q0 * q0 + 2 * q3 * q3 - 1);