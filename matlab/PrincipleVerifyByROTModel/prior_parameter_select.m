close all;
clear;
%%
[X,Y] = meshgrid(logspace(0.01,2.2,300),0.01:0.01:1);

Z = exp(psi(Y) - psi(1)-X)./(exp(psi(Y) - psi(1)-X)+exp(psi(1-Y) - psi(1)));
figure;
% Z = peaks(X,Y);
C = X.*Y;
surfc(X,Y,Z);
colorbar;
figure;
contour(X,Y,Z);
