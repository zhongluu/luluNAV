clear;
close all;

load("TCNormtime1807I.mat");
load("VBTCtime1807I.mat");
load("LCCKFtime1807I_R.mat");
load("TCNormtime1807I_R.mat");
TCCHItime1807I_R = load("TCCHItime1807I_R.mat");
VBTCtime1807I_R = load("VBTCtime1807I_R.mat");
VBTCtime1807I_detail_R = load("VBTCtime1807I_detail_R.mat");
VBNormTC1807I = load("VBNormTCtime1807I_R.mat");
VBNormTC1807I_detail_R = load("VBNormTCtime1807I_detail_R.mat");
NOVEL1807 = load("RSTEKFandMCEKFtime1807.mat");

winEXII = 473;% 1807_I window len

meanLCCKF1807I = mean(LCCKFtime1807I_R);
meanTCNorm1807I = mean(TCNormtime1807I_R);
meanTCCHI1807I = mean(TCCHItime1807I_R.TCCHItime);
meanVBTC1807I = mean(VBTCtime1807I_R.VBTCtime);
meanRSTTC1807I = mean(NOVEL1807.RSTEKFtime);
meanMCTC1807I = mean(NOVEL1807.MCEKFTime);
meanVBNormTC1807I = mean(VBNormTC1807I.VBNormTCtime);
meanVBNormTC1807IDetail = mean(VBNormTC1807I_detail_R.VBNormTCtime);

meanVBTC1807IDetail = 0; m=length(VBTCtime1807I_detail_R.VBTCtime);
for i = 1: m
    meanVBTC1807IDetail = meanVBTC1807IDetail + VBTCtime1807I_detail_R.VBTCtime(i).alltime;
end
meanVBTC1807IDetail = meanVBTC1807IDetail/ m;

f1 = figure('Name', 'Time consumption comparison');
f1.Units = "inches";
f1.Position = [6,3,5.833333333333332,7];
subplot(5,1,1); area(VBTCtime1807I.VarName3(1:winEXII)*1000); hold on;
area(TCNormtime1807I.File(1:winEXII)*1000); hold on;
title(["Time Consumption in real embedded CPU of EX II"],'Fontsize',10,'Interpreter','latex');
ylabel(['Elapse $(ms)$'],'Interpreter','latex');
xlabel(['Running time $t$ (s)'],'Interpreter','latex');
legend( 'Proposed Method','TC-EKF');
subplot(5,1,[2,3]);
X = categorical({'EX II'});
b1 = barh(X,[meanLCCKF1807I,meanTCNorm1807I,meanTCCHI1807I,meanRSTTC1807I,meanMCTC1807I,meanVBTC1807I,meanVBNormTC1807I]*1000);
legend('LC-CKF','TC-EKF','TC-EKFwithChi','RSTEKF','MCEKF', 'Proposed Method(SEQ)', 'Proposed Method(Normal)');
xlabel(['Average consumption time in Matlab $(ms)$'],'Interpreter','latex');
% title(["Average consumption time in Matlab"],'Fontsize',10,'Interpreter','latex');
sgtitle("\textbf{Time consumption in Real \& Matlab of Tightly Coupled navigation}",'Fontsize',10,'Interpreter','Latex');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
subplot(5,1,4);
X = categorical({'Normal','SEQ'});
b2=barh(X,[meanVBNormTC1807IDetail, meanVBTC1807IDetail]*1000,'FaceColor','flat');hold on;
xlabel(['Average Kalman update consumption time in Matlab for proposed method $(ms)$'],'Interpreter','latex');
ax=gca; ax.Position = [0.13,0.26,0.775,0.11]; ax.XGrid = 'on';ax.XMinorGrid = 'on'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
b2.CData(2,:) = [0.30,0.75,0.93]; b2.CData(1,:) = [0.64,0.08,0.18];
subplot(5,1,5);
b3=barh(X,[VBNormTC1807I_detail_R.VBNormTCtime(348), VBTCtime1807I_detail_R.VBTCtime(348).alltime]*1000,'FaceColor','flat');hold on;
b3.CData(2,:) = [0.30,0.75,0.93]; b3.CData(1,:) = [0.64,0.08,0.18];
w2 = .5;
barh(X,[zeros(size(VBTCtime1807I_detail_R.VBTCtime(348).onetime))'; ...
        VBTCtime1807I_detail_R.VBTCtime(348).onetime']*1000,w2,'stacked'); hold off;
ax=gca; ax.Position = [0.13,0.1,0.775,0.11]; ax.XGrid = 'on';ax.XMinorGrid = 'on'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
xlabel(['Single Kalman update consumption time in Matlab for proposed method at 348 $s$ $(ms)$'],'Interpreter','latex');
sgtitle("\textbf{Time consumption in Real \& Matlab of Tightly Coupled navigation}",'Fontsize',10,'Interpreter','Latex');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


