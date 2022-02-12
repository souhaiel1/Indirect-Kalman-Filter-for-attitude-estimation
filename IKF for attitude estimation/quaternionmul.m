function [r] = quaternionmul(p,q)
% function [r] = quaternionmul(p,q)

p0 = p(1);
p1 = p(2);
p2 = p(3);
p3 = p(4);

M = [ p0 , -p1 , -p2 , -p3 ; ...
      p1 , p0 , -p3 , p2 ; ...
      p2 , p3 , p0, -p1 ; ...
      p3 , -p2, p1 , p0];

v = [ q(1) ; q(2) ; q(3) ; q(4) ];

r = M * v;