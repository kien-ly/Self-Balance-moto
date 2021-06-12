m1 = 3;
m2 = 1;
l1 = 1;
l2 = 2;
I1 = 1;
I2 = 1;

g = 9.8;

D = m1*(l1^2) + m2*(l2^2) + I1;

A = (m1*l1 + m2*l2)*g / D;
B = -1 / D;

thongso = [m1 m2 l1 l2 I1 I2]';

% simulate with biased parameters
deviation = 0.2;
thongsodosai = (1 + (deviation*rand(6, 1) - deviation/2)) .* thongso;
disp(thongsodosai);

m1s = thongsodosai(1);
m2s = thongsodosai(2);
l1s = thongsodosai(3);
l2s = thongsodosai(4);
I1s = thongsodosai(5);
I2s = thongsodosai(6);

Ds = m1s*(l1s^2) + m2s*(l2s^2) + I1s;

As = (m1s*l1s + m2s*l2s)*g / Ds;
Bs = -1 / Ds;

disp([A B D]');
disp([As Bs Ds]');