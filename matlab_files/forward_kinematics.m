clear;
syms q1 q2 q3 q4;
l0=36.076;
l1=60.25;
l2=130.2;
l3=124;
l4=133.4;
q=1.39;

A2=[cos(q1) 0 -sin(q1) 0;
    sin(q1) 0 cos(q1) 0;
    0 -1 0 l0+l1;
    0 0 0 1];

A3=[cos(q2-q) -sin(q2-q) 0 l2*cos(q2-q);
    sin(q2-q) cos(q2-q) 0 l2*sin(q2-q);
    0 0 1 0;
    0 0 0 1];

A4=[cos(q3+q) -sin(q3+q) 0 l3*cos(q3+q);
    sin(q3+q) cos(q3+q) 0 l3*sin(q3+q);
    0 0 1 0;
    0 0 0 1];

A5=[cos(q4) -sin(q4) 0 l4*cos(q4);
    sin(q4) cos(q4) 0 l4*sin(q4);
    0 0 1 0;
    0 0 0 1];
H1O=A2;
H2O=A2*A3;
H3O=A2*A3*A4;
H4O=H3O*A5;

