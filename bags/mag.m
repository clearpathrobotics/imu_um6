load "mag1.txt" 
load "rpy1.txt" 
load "rpy2.txt" 
load "mag2.txt" 

rpy1=rpy1(440:630,:);
mag1=mag1(440:630,:);
rpy2=rpy2(110:250,:);
mag2=mag2(110:250,:);


% figure(1)
% plot(rpy1(:,3),"-k")
% figure(2)
% plot(rpy2(:,3),"-k")
% figure(3)
%  scatter3(mag1(:,2),mag1(:,3),mag1(:,4))
% figure(4)
%  scatter3(mag2(:,2),mag2(:,3),mag2(:,4))


 M=mean([mag1(:,2:4);mag2(:,2:4)])
m1 = mag1(:,2:4)-ones(size(mag1,1),1)*M; 
m2 = mag2(:,2:4)-ones(size(mag2,1),1)*M;

plot3(m1(:,1),m1(:,2),m1(:,3),'r*', m2(:,1),m2(:,2),m2(:,3), "b*")

% http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf

m=[m1;m2];
S=sum(m.*m)
Suu = S(1)
Svv = S(2)
Suv = m(:,1)' * m(:,2);
S=sum(m.*m.*m)
Suuu = sum(m(:,1) .* m(:,1) .* m(:,1));
Suvv = sum(m(:,1) .* m(:,2) .* m(:,2));
Svuu = sum(m(:,1) .* m(:,1) .* m(:,2));
Svvv = sum(m(:,2) .* m(:,2) .* m(:,2));
U = (1/2)*inv([Suu Suv;Suv Svv])*[Suuu+Suvv;Svvv+Svuu];
N = length(m);
R2 = U(1)^2 + U(2)^2 + (Suu + Svv)/N;
R = sqrt(R2);

[U(1), U(2), R]

t=0:0.06:2*pi;
plot3(m1(:,1),m1(:,2),m1(:,3),'r*', m2(:,1),m2(:,2),m2(:,3), "b*", U(1)+R*cos(t), U(2)+R*sin(t), zeros(size(t)), 'b-')
view(2)

% Data for KF in office: R = 155.880, Xc = 208.292 Yc = -70.454 Zc = -224.979

