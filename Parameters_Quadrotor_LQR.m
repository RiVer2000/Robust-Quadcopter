m = 0.18;
g = 9.81;
Ix = 0.00025;
Iy = 0.000432;
Iz = 0.0003738;
d = 0.086;
kftx = 0.1;
kfty = 0.1;
kftz = 0.1;
A = zeros(12);
B = zeros(12,4);
v = [1,1,1,1,1,1];
A = diag(v,6);
A(8,4)= -g;
A(7,5)= g;
A(7,7)= -kftx/m;
A(8,8)= -kfty/m;
A(9,9)= -kftz/m;
B(9,1)= 1/m;
B(10,2)= d/Ix;
B(11,3)= d/Iy;
B(12,4)= d/Iz;
Q = zeros(12);
v1 = [100,100,100,0.01,0.01,0.1,10,10,10,0.01,0.01,0.1];
Q = diag(v1);
R = zeros(4);
v2 = [1,1,1,1];
R = diag(v2);
[K,S,eig] = lqr(A,B,Q,R);
