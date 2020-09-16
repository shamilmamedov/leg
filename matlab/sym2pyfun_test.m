clc; clear all; close all;

A = sym(zeros(2,2));

q = sym('q', [2,1], 'real');
syms a b  real

A(1,1) = q(1) + q(2);
A(1,2) = q(1)^2;
A(2,1) = q(1)^2;
A(2,2) = sqrt(q(1) + q(2));


fprintMatPy2('test', {'q1', 'q2'}, A, 2)