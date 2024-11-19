close all;
clear;
glvs;
load("../EXNAVDATA_1031_1807_I.mat");
load("../EXNAVDATA_1031_1807_I_SPP.mat");
load("EX_AHRS_1031_175203.mat");
load("TC21Norm_1807_I.mat");
load("VB21seq_1807_I.mat");
load("TC21CHI_1807_I.mat");
load("LCCKF_1807_I.mat");
load("SPANTRJ_1031_1750.mat");
load("BiasFixed.mat"); % compensate the fixed bias for ins
load("BiasSPANFixed.mat");
avp(:,1:3) = rad2deg(avp(:,1:3));


%%  AHRS comp
close all;
sSPAN = 74 + 19; % 1807_I start timing 
sFOG4MINE = 1931+148; % 1807_I start timing compensate delay between AHRS and Mine 
sFOG4SPAN = 1931+163; % 1807_I start timing compensate delay between AHRS and SPAN
win = 473;% 1807_I window len


errorS = 150;
tIntervalFOG = 100;
tIntervalSPAN = 1;
eFOG4MINE = sFOG4MINE + win*tIntervalFOG - 1;
eFOG4SPAN = sFOG4SPAN + win*tIntervalFOG - 1;
indexFOG4MINE = sFOG4MINE:tIntervalFOG:eFOG4MINE;
indexFOG4SPAN = sFOG4SPAN:tIntervalFOG:eFOG4SPAN;
eSPAN = sSPAN + win*tIntervalSPAN - 1;
indexSPAN = sSPAN:tIntervalSPAN:eSPAN;
len = length(indexFOG4MINE);
AHRS_res4MINE = zeros(len,9);

AHRS_res4MINE(:,1) = FOG_pitch(indexFOG4MINE); AHRS_res4MINE(:,2) = FOG_roll(indexFOG4MINE); AHRS_res4MINE(:,3) = -FOG_heading(indexFOG4MINE);
AHRS_res4MINE(:,4) = FOG_vn_E(indexFOG4MINE); AHRS_res4MINE(:,5) = FOG_vn_N(indexFOG4MINE); AHRS_res4MINE(:,6) = -FOG_vn_D(indexFOG4MINE);
AHRS_res4MINE(:,7) = deg2rad(FOG_pos_lat(indexFOG4MINE)); AHRS_res4MINE(:,8) = deg2rad(FOG_pos_lon(indexFOG4MINE)); AHRS_res4MINE(:,9) = FOG_pos_high(indexFOG4MINE);

AHRS_res4SPAN = zeros(len,9);

AHRS_res4SPAN(:,1) = FOG_pitch(indexFOG4SPAN); AHRS_res4SPAN(:,2) = FOG_roll(indexFOG4SPAN); AHRS_res4SPAN(:,3) = -FOG_heading(indexFOG4SPAN);
AHRS_res4SPAN(:,4) = FOG_vn_E(indexFOG4SPAN); AHRS_res4SPAN(:,5) = FOG_vn_N(indexFOG4SPAN); AHRS_res4SPAN(:,6) = -FOG_vn_D(indexFOG4SPAN);
AHRS_res4SPAN(:,7) = deg2rad(FOG_pos_lat(indexFOG4SPAN)); AHRS_res4SPAN(:,8) = deg2rad(FOG_pos_lon(indexFOG4SPAN)); AHRS_res4SPAN(:,9) = FOG_pos_high(indexFOG4SPAN);

SPAN_res = [rad2deg(trj.avp(indexSPAN,1:3)),trj.avp(indexSPAN,4:end)];
SPAN_res(:,3) = -SPAN_res(:,3);

SPAN_res(find(SPAN_res(:,3)<-180),3) = SPAN_res(find(SPAN_res(:,3)<-180),3) + 360;
AHRS_res4MINE(find(AHRS_res4MINE(:,3)<-180),3) = AHRS_res4MINE(find(AHRS_res4MINE(:,3)<-180),3) + 360;
AHRS_res4SPAN(find(AHRS_res4SPAN(:,3)<-180),3) = AHRS_res4SPAN(find(AHRS_res4SPAN(:,3)<-180),3) + 360;

% convert yaw to an uniform scope
LCCKFYaw = avp(1:win,3) + biasYaw;
LCCKFYaw(find(LCCKFYaw(1:win)>178)) = LCCKFYaw(find(LCCKFYaw(1:win)>178)) - 360;
TCCHIYaw = TCCHIestRes(1:win,3) + biasYaw;
TCCHIYaw(find(TCCHIYaw(1:win)>178)) = TCCHIYaw(find(TCCHIYaw(1:win)>178)) - 360;
seqestResYaw = seqestRes(1:win, 3) + biasYaw;
seqestResYaw(find(seqestResYaw(1:win)>178)) = seqestResYaw(find(seqestResYaw(1:win)>178)) - 360;
VBseqYaw = VBseq(1:win, 3) + biasYaw;
VBseqYaw(find(VBseqYaw(1:win)>178)) = VBseqYaw(find(VBseqYaw(1:win)>178)) - 360;
SPANYaw = SPAN_res(1:win, 3) + biasSPANYaw;
SPANYaw(find(SPANYaw(1:win)>177)) = SPANYaw(find(SPANYaw(1:win)>177)) - 360;
SPANYaw(find(SPANYaw(1:win)<-180)) = SPANYaw(find(SPANYaw(1:win)<-180)) + 360;
% attitude
figure('Name', 'attitude');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [ seqestRes(1:win, 1) + biasPitch,VBseq(1:win,1) + biasPitch,AHRS_res4MINE(1:win,1), SPAN_res(1:win,1),avp(1:win,1)+ biasPitch,TCCHIestRes(1:win,1)+ biasPitch]); hold on; % pitch
subplot(3, 1, 2); plot(seqestRes(1:win, end), [ seqestRes(1:win, 2) + biasRoll,VBseq(1:win,2) + biasRoll,AHRS_res4MINE(1:win,2), SPAN_res(1:win,2),avp(1:win,2) + biasRoll,TCCHIestRes(1:win,2)+ biasRoll]); hold on; % yaw
subplot(3, 1, 3); plot(seqestRes(1:win, end), [ seqestResYaw,VBseqYaw,AHRS_res4MINE(1:win,3), SPANYaw,LCCKFYaw,TCCHIYaw]); hold off; % roll
legend('TCEKF','VBTC','AHRS','SPAN','LCwithCKF', 'TCChi');
xlabel(['$t$(s)'],'Interpreter','latex');

% velocity
figure('Name', 'velocity');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [seqestRes(1:win, 4),VBseq(1:win,4),AHRS_res4MINE(1:win,4), SPAN_res(1:win,4),avp(1:win,4),TCCHIestRes(1:win,4)]); hold on; % east
subplot(3, 1, 2); plot(seqestRes(1:win, end), [seqestRes(1:win, 5),VBseq(1:win,5),AHRS_res4MINE(1:win,5), SPAN_res(1:win,5),avp(1:win,5),TCCHIestRes(1:win,5)]); hold on; % north
subplot(3, 1, 3); plot(seqestRes(1:win, end), [seqestRes(1:win, 6),VBseq(1:win,6),AHRS_res4MINE(1:win,6), SPAN_res(1:win,6),avp(1:win,6),TCCHIestRes(1:win,6)]); hold on; % up
legend('TCEKF','VBTC','AHRS','SPAN','LCwithCKF', 'TCChi');
% position
figure('Name', 'position');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [seqestRes(1:win, 7) + biasLat,VBseq(1:win,7) + biasLat,AHRS_res4MINE(1:win,7), SPAN_res(1:win,7),avp(1:win,7),TCCHIestRes(1:win,7)+ biasLat]); hold on; % latitude
subplot(3, 1, 2); plot(seqestRes(1:win, end), [seqestRes(1:win, 8) + biasLon,VBseq(1:win,8) + biasLon,AHRS_res4MINE(1:win,8), SPAN_res(1:win,8),avp(1:win,8),TCCHIestRes(1:win,8)+ biasLon]); hold on; % longitude
subplot(3, 1, 3); plot(seqestRes(1:win, end), [seqestRes(1:win, 9) + biasHigh,VBseq(1:win,9) + biasHigh,AHRS_res4MINE(1:win,9), SPAN_res(1:win,9),avp(1:win,9) + biasHigh,TCCHIestRes(1:win,9)+ biasHigh]); hold on; % height
xlabel(['$t$(s)'],'Interpreter','latex');
legend('TCEKF','VBTC','AHRS','SPAN','LCwithCKF', 'TCChi');

% %% error

errSPAN = SPAN_res(1:win,1:9) - AHRS_res4SPAN;

errTCNorm = seqestRes(1:win,1:9) - AHRS_res4MINE;
errVBTC = VBseq(1:win,1:9) - AHRS_res4MINE;
errLCCKF = avp(1:win,1:9) - AHRS_res4MINE;
errTCCHI = TCCHIestRes(1:win,1:9) - AHRS_res4MINE;

errLCCKF(1:win, 3) = LCCKFYaw - AHRS_res4MINE(1:win,3);
errTCCHI(1:win, 3) = TCCHIYaw - AHRS_res4MINE(1:win,3);
errTCNorm(1:win, 3) = seqestResYaw - AHRS_res4MINE(1:win,3);
errVBTC(1:win, 3) = VBseqYaw - AHRS_res4MINE(1:win,3);
errSPAN(1:win, 3) = SPANYaw - AHRS_res4SPAN(1:win,3);

RN = glv.Re./sqrt(1-glv.e2*sin(AHRS_res4MINE(1:win,8)).^2).*cos(AHRS_res4MINE(1:win,8));
RM = RN*(1-glv.e2)./(1-glv.e2*sin(AHRS_res4MINE(1:win,8)).^2);

f1 = figure('Name', 'attitude error');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [errLCCKF(1:win,1)+ biasPitch, errTCNorm(1:win, 1) + biasPitch,errTCCHI(1:win,1)+ biasPitch, errSPAN(1:win,1)+biasSPANPitch, errVBTC(1:win,1) + biasPitch]); hold on; % pitch
ylabel(['$\delta \phi_E$ ($\circ$)'],'Interpreter','latex');
subplot(3, 1, 2); plot(seqestRes(1:win, end), [errLCCKF(1:win,2)+ biasRoll, errTCNorm(1:win, 2) + biasRoll,errTCCHI(1:win,2)+biasRoll, errSPAN(1:win,2)+biasSPANRoll,errVBTC(1:win,2) + biasRoll]); hold on; % yaw
ylabel(['$\delta \phi_N$ ($\circ$)'],'Interpreter','latex');
subplot(3, 1, 3); plot(seqestRes(1:win, end), [errLCCKF(1:win,3), errTCNorm(1:win, 3),errTCCHI(1:win,3), errSPAN(1:win,3) ,errVBTC(1:win,3)]); hold off; % roll
ylabel(['$\delta \phi_U$ ($\circ$)'],'Interpreter','latex');
xlabel(['$t$(s)'],'Interpreter','latex');
sgtitle("\textbf{attitude estimation error - EX II}",'Fontsize',10,'Interpreter','Latex');
legend('LC-CKF','TC-EKF','TC-EKFwithChi','SPAN', 'Proposed Method');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
% velocity
f2 = figure('Name', 'velocity error');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [errLCCKF(1:win,4),errTCNorm(1:win, 4),errTCCHI(1:win,4), errSPAN(1:win,4),errVBTC(1:win,4)]); hold on; % east
ylabel(['$\delta v_E$ ($m/s$)'],'Interpreter','latex');
subplot(3, 1, 2); plot(seqestRes(1:win, end), [errLCCKF(1:win,5),errTCNorm(1:win, 5),errTCCHI(1:win,5), errSPAN(1:win,5),errVBTC(1:win,5)]); hold on; % north
ylabel(['$\delta v_N$ ($m/s$)'],'Interpreter','latex');
subplot(3, 1, 3); plot(seqestRes(1:win, end), [errLCCKF(1:win,6),errTCNorm(1:win, 6),errTCCHI(1:win,6), errSPAN(1:win,6),errVBTC(1:win,6)]); hold on; % up
ylabel(['$\delta v_U$ ($m/s$)'],'Interpreter','latex');
xlabel(['$t$(s)'],'Interpreter','latex');
sgtitle("\textbf{velocity estimation error - EX II}",'Fontsize',10,'Interpreter','Latex');
legend('LC-CKF','TC-EKF','TC-EKFwithChi','SPAN', 'Proposed Method');
pos = get(f2,'Position');
set(f2,'Units','Inches');
set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
% position
f3 = figure('Name', 'position error');
subplot(3, 1, 1); plot(seqestRes(1:win, end), [errLCCKF(1:win,7)+ biasLat,errTCNorm(1:win, 7) + biasLat,errTCCHI(1:win,7)+ biasLat, errSPAN(1:win,7),errVBTC(1:win,7) + biasLat].*RN(1:win)); hold on; % latitude
ylabel(['$\delta L$ ($m$)'],'Interpreter','latex');
subplot(3, 1, 2); plot(seqestRes(1:win, end), [errLCCKF(1:win,8),errTCNorm(1:win, 8) + biasLon,errTCCHI(1:win,8)+ biasLon, errSPAN(1:win,8),errVBTC(1:win,8) + biasLon].*RM(1:win)); hold on; % longitude
ylabel(['$\delta \lambda$ ($m$)'],'Interpreter','latex');
subplot(3, 1, 3); plot(seqestRes(1:win, end), [errLCCKF(1:win,9) + biasHigh,errTCNorm(1:win, 9) + biasHigh,errTCCHI(1:win,9)+ biasHigh, errSPAN(1:win,9),errVBTC(1:win,9) + biasHigh]); hold on; % height
ylabel(['$\delta h$ ($m$)'],'Interpreter','latex');
xlabel(['$t$(s)'],'Interpreter','latex');
sgtitle("\textbf{position estimation error - EX II}",'Fontsize',10,'Interpreter','Latex');
legend('LC-CKF','TC-EKF','TC-EKFwithChi','SPAN', 'Proposed Method');
pos = get(f3,'Position');
set(f3,'Units','Inches');
set(f3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f4 = figure('Name', 'Proposed Method vs Chi');
subplot(211);colormap(turbo);VBR = pcolor(Rt); VBR.LineStyle=":";VBR.LineWidth = 0.1;
title(["Proposed Method"],'Interpreter','latex');
ylabel(['Satellites'],'Interpreter','latex');
subplot(212);colormap([[1 1 1];turbo(1000000);]);CHIR = pcolor(ChiIndex); CHIR.LineStyle=":";CHIR.LineWidth = 0.1;
title(["$\chi^2$ working index"],'Interpreter','latex');
ylabel(['Satellites'],'Interpreter','latex');
xlabel(['$t$(s)'],'Interpreter','latex');
sgtitle("\textbf{Measurement Estimation}",'Fontsize',10,'Interpreter','Latex');
pos = get(f4,'Position');
set(f4,'Units','Inches');
set(f4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f5 = figure('Name', 'Trj');
plot(rad2deg(AHRS_res4MINE(1:win, 8)),rad2deg(AHRS_res4MINE(1:win, 7)));hold on;
plot(rad2deg(avp(1:win, 8)),rad2deg(avp(1:win,7)+ biasLat));hold on;
plot(rad2deg(seqestRes(1:win, 8)+ biasLon),rad2deg(seqestRes(1:win, 7)+ biasLat));hold on;
plot(rad2deg(TCCHIestRes(1:win, 8)+ biasLon),rad2deg(TCCHIestRes(1:win, 7)+ biasLat));hold on;
plot(rad2deg(SPAN_res(1:win, 8)),rad2deg(SPAN_res(1:win, 7)));hold on;
plot(rad2deg(VBseq(1:win, 8)+ biasLon),rad2deg(VBseq(1:win, 7)+ biasLat));hold on;
title(["EX II trajectory from various methods"],'Fontsize',10,'Interpreter','latex');
xlabel(['Longitude $(\circ)$'],'Interpreter','latex');
ylabel(['Latitude $(\circ)$'],'Interpreter','latex');
legend('AHRS','LC-CKF','TC-EKF','TC-EKFwithChi','SPAN', 'Proposed Method');
pos = get(f5,'Position');
set(f5,'Units','Inches');
set(f5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

LCCKF_RMSE_Pitch = rmse(errLCCKF(errorS:end,1) + biasPitch,0);
LCCKF_RMSE_Roll = rmse(errLCCKF(errorS:end,2) + biasRoll,0);
LCCKF_RMSE_Yaw = rmse(errLCCKF(errorS:end,3),0);

TCCHI_RMSE_Pitch = rmse(errTCCHI(errorS:end,1) + biasPitch,0);
TCCHI_RMSE_Roll = rmse(errTCCHI(errorS:end,2) + biasRoll,0);
TCCHI_RMSE_Yaw = rmse(errTCCHI(errorS:end,3),0);

TCNORM_RMSE_Pitch = rmse(errTCNorm(errorS:end,1) + biasPitch,0);
TCNORM_RMSE_Roll = rmse(errTCNorm(errorS:end,2) + biasRoll,0);
TCNORM_RMSE_Yaw = rmse(errTCNorm(errorS:end,3),0);

VB_RMSE_Pitch = rmse(errVBTC(errorS:end,1) + biasPitch,0);
VB_RMSE_Roll = rmse(errVBTC(errorS:end,2) + biasRoll,0);
VB_RMSE_Yaw = rmse(errVBTC(errorS:end,3),0);

SPAN_RMSE_Pitch = rmse(errSPAN(errorS:end,1)+ biasSPANPitch,0);
SPAN_RMSE_Roll = rmse(errSPAN(errorS:end,2)+biasSPANRoll,0);
SPAN_RMSE_Yaw = rmse(errSPAN(errorS:end,3),0);

LCCKF_RMSE_VelE = rmse(errLCCKF(errorS:end,4),0);
LCCKF_RMSE_VelN = rmse(errLCCKF(errorS:end,5),0);
LCCKF_RMSE_VelU = rmse(errLCCKF(errorS:end,6),0);

TCCHI_RMSE_VelE = rmse(errTCCHI(errorS:end,4),0);
TCCHI_RMSE_VelN = rmse(errTCCHI(errorS:end,5),0);
TCCHI_RMSE_VelU = rmse(errTCCHI(errorS:end,6),0);

TCNORM_RMSE_VelE = rmse(errTCNorm(errorS:end,4),0);
TCNORM_RMSE_VelN = rmse(errTCNorm(errorS:end,5),0);
TCNORM_RMSE_VelU = rmse(errTCNorm(errorS:end,6),0);

VB_RMSE_VelE = rmse(errVBTC(errorS:end,4),0);
VB_RMSE_VelN = rmse(errVBTC(errorS:end,5),0);
VB_RMSE_VelU = rmse(errVBTC(errorS:end,6),0);

SPAN_RMSE_VelE = rmse(errSPAN(errorS:end,4),0);
SPAN_RMSE_VelN = rmse(errSPAN(errorS:end,5),0);
SPAN_RMSE_VelU = rmse(errSPAN(errorS:end,6),0);


LCCKF_RMSE_Lat = rmse((errLCCKF(errorS:end,7) + biasLat).*RN(errorS:end),0);
LCCKF_RMSE_Lon = rmse((errLCCKF(errorS:end,8)).*RM(errorS:end),0);
LCCKF_RMSE_High = rmse(errLCCKF(errorS:end,9) + biasHigh,0);

TCCHI_RMSE_Lat = rmse((errTCCHI(errorS:end,7) + biasLat).*RN(errorS:end),0);
TCCHI_RMSE_Lon = rmse((errTCCHI(errorS:end,8)+ biasLon).*RM(errorS:end),0);
TCCHI_RMSE_High = rmse(errTCCHI(errorS:end,9) + biasHigh,0);

TCNORM_RMSE_Lat = rmse((errTCNorm(errorS:end,7) + biasLat).*RN(errorS:end),0);
TCNORM_RMSE_Lon = rmse((errTCNorm(errorS:end,8) + biasLon).*RM(errorS:end),0);
TCNORM_RMSE_High = rmse(errTCNorm(errorS:end,9) + biasHigh,0);

VB_RMSE_Lat = rmse((errVBTC(errorS:end,7) + biasLat).*RN(errorS:end),0);
VB_RMSE_Lon = rmse((errVBTC(errorS:end,8) + biasLon).*RM(errorS:end),0);
VB_RMSE_High = rmse(errVBTC(errorS:end,9) + biasHigh,0);

SPAN_RMSE_Lat = rmse(errSPAN(errorS:end,7).*RN(errorS:end),0);
SPAN_RMSE_Lon = rmse(errSPAN(errorS:end,8).*RM(errorS:end),0);
SPAN_RMSE_High = rmse(errSPAN(errorS:end,9),0);
