clear;
close all;

load("TCNormtime1807I.mat");
load("VBTCtime1807I.mat");
load("LCCKFtime1807I_R.mat");
load("TCNormtime1807I_R.mat");
load("TCCHItime1807I_R.mat");
load("VBTCtime1807I_R.mat");

winEXII = 473;% 1807_I window len
IMU = 1/125;

meanLCCKF1807I = mean(LCCKFtime1807I_R);
meanTCNorm1807I = mean(TCNormtime1807I_R);
meanTCCHI1807I = mean(TCCHItime1807I_R);
meanVBTC1807I = mean(VBTCtime1807I_R);


f1 = figure('Name', 'Time consumption comparison');

subplot(3,1,1); area(VBTCtime1807I.VarName3(1:winEXII)*1000); hold on;
area(TCNormtime1807I.File(1:winEXII)*1000); hold on;
title(["Time Consumption in real embedded CPU of EX II"],'Fontsize',10,'Interpreter','latex');
ylabel(['Elapse $(ms)$'],'Interpreter','latex');
xlabel(['Running time $t$ (s)'],'Interpreter','latex');
legend( 'Proposed Method','TC-EKF');
subplot(3,1,[2,3]);
X = categorical({'EX II'});
b1 = barh(X,[meanLCCKF1807I,meanTCNorm1807I,meanTCCHI1807I,meanVBTC1807I]*1000);
legend('LC-CKF','TC-EKF','TC-EKFwithChi', 'Proposed Method');
xlabel(['Average consumption time in Matlab $(ms)$'],'Interpreter','latex');
% title(["Average consumption time in Matlab"],'Fontsize',10,'Interpreter','latex');
sgtitle("\textbf{Time consumption in Real \& Matlab of Tightly Coupled navigation}",'Fontsize',10,'Interpreter','Latex');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


