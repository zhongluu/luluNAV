% LULUNAV
% Copyright (C) {{ 2024 }}  {{ yulu_zhong }}

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Affero General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.

% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Affero General Public License for more details.

% You should have received a copy of the GNU Affero General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% Copyright(c) 2024, by Yulu Zhong. All rights reserved.
% Key Laboratory of Micro-Inertial Instrument and Advanced Navigation Technology of Ministry of Education,
% Southeast University, NanJing, P.R.China 12/11/2024
% based on psins toolbox from http://www.psins.org.cn/
% version:psins241109.rar
% or psins240513.rar
% Acknowledge: Gongmin Yan and Kailong Li.
close all;
clear;
glvs;
load("../EXNAVDATA_1031_1807_I.mat");
load("../EXNAVDATA_1031_1807_I_SPP.mat");
tmpposL = deg2rad(posL(1,1:2));
trj.avp0=[deg2rad([-1.178971197228000,-4.539559178293000,-8.5]),0,0,0, tmpposL,posL(1,3)]; % for 1807
nn = 1; ts = 1/125; nts = 1*ts;
nts2 = nts / 2;
% sensor's deviation defined -- same as reference
TGNSS = 1;
deviationOfGyroV = rad2deg(4.3633e-05 * 60); % rad/sqrt(Hz) --> deg/sqrt(h)
deviationOfGyroU = 9.1989e-7; % rad/s/sqrt(Hz)
deviationOfAccV = 0.0012 / glv.ug; % m/s^2/sqrt(Hz) --> ug/sqrt(Hz)
deviationOfAccU = 6e-5; % m/s^2/s^2/sqrt(Hz)
initialAttiErr = 10 * 60 * ones(1, 3); % deg --> arcmin
initialVelErr = [0.005,0.005,0.005];
initialPosErr = zeros(1, 3);
initialBiasG = 250; % deg/hr
initialBiasA = 0.073; % m/s^2 
% GNSS err 
SIS_err_SD = 1; % Signal in space error SD (m)
zenith_iono_err_SD = 2; % Zenith ionosphere error SD (m) 
zenith_trop_err_SD = 0.2; % Zenith troposphere error SD (m) 
% SIS_err_SD = 0; % Signal in space error SD (m)
% zenith_iono_err_SD = 0; % Zenith ionosphere error SD (m) 
% zenith_trop_err_SD = 0; % Zenith troposphere error SD (m) 
code_track_err_SD = 1; % Code tracking error SD (m) 
rate_track_err_SD = 0.2; % Range rate tracking error SD (m/s) 
rx_clock_offset = 10000; % Receiver clock offset at time=0 (m);
rx_clock_drift = 10; % Receiver clock drift at time=0 (m/s);
clock_freq_PSD = rate_track_err_SD; % Receiver clock frequency-drift PSD (m^2/s^3)
clock_phase_PSD = code_track_err_SD; % Receiver clock phase-drift PSD (m^2/s)
pseudo_range_SD = 5.5; % Pseudo-range measurement noise SD (m)
range_rate_SD = 0.01; % Pseudo-range rate measurement noise SD (m/s)

%% init

%% a direct Multiplicative extended kalman filter : Kg and Ka are omitted
% reference: Li Fangneng, Chang Lubin. "MEKF With Navigation Frame Attitude Error Parameterization for INS/GPS"
%            Li Kailong,Lu Xin, Li Wenkui. "Nonlinear Error Model Based on Quaternion for the INS: Analysis and Comparison"

% init MEKF
davp0 = avperrset(initialAttiErr, initialVelErr, initialPosErr);
% initavp = avpadderr(trj.avp0, davp0);
initavp = trj.avp0';
opt_q = a2qua(initavp(1:3)); % the actual quaternion for filter
pos = initavp(7:9);
vel = initavp(4:6);
esteth = ethinit(pos, vel);
biasG = zeros(3, 1);
biasA = zeros(3, 1);
lever = zeros(3,1);
dly = 0; 
deltaS = zeros(3, 1); % nominal quaternion
coffset = 0;
cdrift = 0;
X = [deltaS', vel', pos', biasG', biasA', lever', dly, coffset, cdrift]'; % a 15 state -- aleph0 pos after vel
% filter required parameter
P = diag([(deg2rad(10/3))^2 * ones(1, 3), (0.15/3)^2 * ones(1, 2), (0.15/3)^2, 1e-6^2 * ones(1, 2), (10/3)^2, ...
        (initialBiasG * glv.dph)^2 * ones(1, 3), (initialBiasA)^2 * ones(1, 3), 0.8*ones(1,3), 0.005^2, rx_clock_offset^2, rx_clock_drift^2]); % same as reference

Q =  diag([deg2rad((deviationOfGyroV) / 60)^2 * ones(1, 3), (0.8/60)^2 * ones(1, 3), deg2rad(0.3/3600)^2* ones(1, 3), 0.004^2*ones(1,3) clock_phase_PSD^2, clock_freq_PSD^2]);

R = [pseudo_range_SD^2, range_rate_SD^2];
seqR = pseudo_range_SD^2;
n = length(X);
% KF Init
DKFins = insinit(trj.avp0', ts);
DKFins.lever = [0.1,0.1,0.8]';
DKFins.dly = 0.005;
DKFP = P;
DKFins.alpha = zeros(3,1);

DKFins.cdrift = 0;
DKFins.coffset = 0;
seqDKFins = DKFins;
seqDKFP = DKFP;
% prealloc
len = length(imu);
[estRes, estNormError, realRes] = prealloc(fix(len / nn), length(X) + 1, 7, length(X));
[seqestRes,  seqestNormError, seqrealRes] = prealloc(fix(len / nn), length(X) + 1, 7, length(X));
[sigmaLine3] = prealloc(fix(len / nn),2*length(X));
timebar(nn, len, 'MineMEKFTCNV.');
ki = 1;
ChiRes = zeros(length(GNSS.utc),max(GNSS.no_GNSS(:,1)));

map = containers.Map('KeyType','double','ValueType','any');
for k = 1:nn:len - nn + 1
    k1 = k + nn - 1;
    wvm = imu(k:k1, 1:6);
    t = imu(k1, end);

    [seqDKFins, seqDKFP] = MEKFPredictionTC2(seqDKFins, seqDKFP, Q, wvm, nts);
    %% correction
    if mod(t, TGNSS)==0
        % pick current GNSS measurement
        curGNSS.obs = GNSS.obs(:,:, ki);
        curGNSS.no_GNSS = GNSS.no_GNSS(ki, :);
        curGNSS.prn = GNSS.prn(ki,:);
        %
        if t < 2
            % for Chi running beginning the Chi threshold must set large
            if curGNSS.no_GNSS(1) > 0
                [seqDKFins, seqDKFP] =  MEKFTCNHC(seqDKFins, seqDKFP);
                tic;
                [seqDKFins, seqDKFP, Chi,ChiWork,map] = seqMEKFCorrectionTC2(seqDKFins, seqDKFP, seqR, curGNSS,180,map,t);
                TCCHItime(ki) = toc;
                ChiRes(ki,1:GNSS.no_GNSS(ki, 1)) = Chi;
            end
        else
            [seqDKFins, seqDKFP] =  MEKFTCNHC(seqDKFins, seqDKFP);
            tic;
            [seqDKFins, seqDKFP, Chi,ChiWork,map] = seqMEKFCorrectionTC2(seqDKFins, seqDKFP, seqR, curGNSS,3.81,map,t);
            TCCHItime(ki) = toc;
            ChiRes(ki,1:GNSS.no_GNSS(ki, 1)) = Chi;
%             ChiIndex(ki,1:GNSS.no_GNSS(ki, 1)) = ChiWork;
        end
        % measurement updating completing

        % save filter result
        seqestAtti = rad2deg(q2att(seqDKFins.qnb)); % deg, m/s, m
        seqestRes(ki, :) = [seqestAtti; seqDKFins.vn; seqDKFins.pos; seqDKFins.eb; seqDKFins.db; seqDKFins.lever; seqDKFins.dly; seqDKFins.coffset; seqDKFins.cdrift; t]';

        ki = ki + 1;
    end

    timebar;
end


valueList = values(map);
[~,n] = size(valueList);
ChiIndex = zeros(n,ki);
for i =1:n
    curVBMInfo = valueList{i};
    m = length(curVBMInfo.time);
    for j = 1:m
        ChiIndex(i,curVBMInfo.time(j)) = curVBMInfo.ChiIndex(j);
    end
end

% free unnecessary space for ram
seqestRes(ki:end,:) = [];
TCCHIestRes = seqestRes;
%% plot
% attitude
figure('Name', 'attitude');
subplot(3, 1, 1); plot(seqestRes(:, end), [ seqestRes(:, 1)]); hold on; % pitch
subplot(3, 1, 2); plot(seqestRes(:, end), [ seqestRes(:, 2)]); hold on; % yaw
subplot(3, 1, 3); plot(seqestRes(:, end), [ seqestRes(:, 3)]); hold off; % roll
legend('Estimation');
% velocity
figure('Name', 'velocity');
subplot(3, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 4)]); hold on; % east
subplot(3, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 5)]); hold on; % north
subplot(3, 1, 3); plot(seqestRes(:, end), [seqestRes(:, 6)]); hold on; % up
legend('Estimation');
% position
figure('Name', 'position');
subplot(3, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 7)]); hold on; % latitude
subplot(3, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 8)]); hold on; % longitude
subplot(3, 1, 3); plot(seqestRes(:, end), [seqestRes(:, 9)]); hold on; % height
legend('Estimation');
% gyro bias estimation
figure('Name', 'gyroscope bias');
subplot(3, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 10)]); hold on; % x
subplot(3, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 11)]); hold on; % y
subplot(3, 1, 3); plot(seqestRes(:, end), [seqestRes(:, 12)]); hold on; % z
legend('Estimation');
% acc bias estimation
figure('Name', 'accelerate bias');
subplot(3, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 13)]); hold on; % x
subplot(3, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 14)]); hold on; % y
subplot(3, 1, 3); plot(seqestRes(:, end), [seqestRes(:, 15)]); hold on; % z
legend('Estimation');
% lever estimation
figure('Name', 'lever bias');
subplot(3, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 16)]); hold on; % x
subplot(3, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 17)]); hold on; % y
subplot(3, 1, 3); plot(seqestRes(:, end), [seqestRes(:, 18)]); hold on; % z
legend('Estimation');
% dly estimation
figure('Name', 'dly bias');
plot(seqestRes(:, end), [seqestRes(:, 19)]); hold on; % x
legend('Estimation');
% clock offset and clock drift
figure('Name', 'clock offset and clock drift');
subplot(2, 1, 1); plot(seqestRes(:, end), [seqestRes(:, 20)]); hold on; % x
subplot(2, 1, 2); plot(seqestRes(:, end), [seqestRes(:, 21)]); hold on; % y
legend('True','Estimation');
% trajectory
posL(:,1:2) = deg2rad(posL(:,1:2));
delta = posL(1,:) - seqestRes(1,7:9);
posL(:,1) = posL(:,1) - delta(1);posL(:,2) = posL(:,2) - delta(2);
posL(:,3) = posL(:,3) - delta(3);

figure('Name', 'vehicle tracjectory');
plot3([seqestRes(:, 7)],[seqestRes(:, 8)],seqestRes(:, 9));hold on;
plot3([seqestRes(1, 7)],[seqestRes(1, 8)],seqestRes(1, 9),'^');hold on;
plot3(posL(:,1),posL(:,2),posL(:,3),'*');
legend('TC','start','spp');

figure('Name', 'Chi estimation R');
CHIR = pcolor(ChiIndex); CHIR.LineStyle=":";CHIR.LineWidth = 0.1;
colormap([[1 1 1];turbo(44);]);

% MEKF filter basic function

function [ins, P] = MEKFPredictionTC2(ins, P, Q, wvm, nts)
    % neccessary parameters
    glvs;
    ins = insupdate(ins, wvm);
    ins.coffset = ins.coffset + ins.cdrift * nts;

    Fhi= etm(ins);
    Fhi = [Fhi, zeros(15,6);
            zeros(4,21);
            zeros(1,20),1;
            zeros(1,21)];

    Fhi = eye(size(Fhi)) + Fhi*nts;

    G = [ eye(3),  zeros(3),zeros(3),zeros(3), zeros(3,1), zeros(3,1); %a
           zeros(3),  eye(3),zeros(3),zeros(3), zeros(3,1), zeros(3,1); %v
                        zeros(3,14); %p
            zeros(3,6),eye(3),zeros(3), zeros(3,1), zeros(3,1); %eb
            zeros(3,9),eye(3),zeros(3,1),zeros(3,1); %db
                        zeros(4,14); %lever
                  zeros(1, 12),     1,        0; % corr
                  zeros(1, 12),     0,        1;]; % drift
    % covariance update
    P = Fhi * P * Fhi';
    P = P + G * Q * G' * nts;
    P= (P + P')/2;

end

function [ins, P, ChiRes, Chiindex, map] = seqMEKFCorrectionTC2(ins, P, R, GNSS, ChiTH, map,t)
    lightspeed = 299792458; % Speed of light in m/s
    % get state nums and valid GNSS nums
    [stateNum, ~] = size(P);
    % no_GNSS(2) inidicate current time
    no_GNSS = GNSS.no_GNSS;
    
        prn = GNSS.prn;
        GNSS = GNSS.obs;
    staDly = GNSS(1:no_GNSS(1),5);
    lonoDly = GNSS(1:no_GNSS(1),6);
    tropDly = GNSS(1:no_GNSS(1),7);
    ChiRes = zeros(1,no_GNSS(1));
    Chiindex = zeros(1,no_GNSS(1));

    % cal R

    % get est vel and pos from est ins
    ins = inslever(ins);
    ins.posL = ins.posL-ins.Mpvvn * ins.dly;
    parXYZBLH = Dblh2Dxyz(ins.posL);

    for i = 1 : no_GNSS(1)

        if isKey(map,prn(i))
            curVBMInfo = map(prn(i));
        else
            curVBMInfo.ChiRes = [];
            curVBMInfo.ChiIndex = [];
            curVBMInfo.time = [];
        end


    ins = inslever(ins);
    ins.posL = ins.posL-ins.Mpvvn * ins.dly;
    Pn = ins.posL;
    [Pe, Cen] = blh2xyz(Pn);

    % cal est obs value range and range rate and H matrix

    r_ea_e = Pe; 

        sat_r_es_e = GNSS(i, 2:4); 
        delta_r = sat_r_es_e' - r_ea_e;
        approx_range = sqrt(delta_r' * delta_r);
        u_as_e = delta_r / approx_range;
        Omega_ie = askew([0,0,ins.eth.wie]);
        C_e_I = (eye(3) - Omega_ie * (approx_range / lightspeed));
        delta_r = C_e_I * sat_r_es_e' - r_ea_e;
        obs_meas = sqrt(delta_r' * delta_r) + ins.coffset; % for range

        % cal H matrix

        H = [zeros(1,3), zeros(1,3), u_as_e'*parXYZBLH,  zeros(1,6),u_as_e'*ins.MpvCnb, u_as_e'*ins.Mpvvn  1, 0]; % for range

        rV = GNSS(i,1)+staDly(i) +lonoDly(i) +  tropDly(i);
        innov_cov = H * P * H' + R; 
        cross_corr_cov = P * H'; 
    % standard SPKF -- deriv K
        KGain = cross_corr_cov * invbc(innov_cov); 
    % measurement updating
        innov = (rV - obs_meas); 
        Chi = innov*innov/innov_cov;
        ChiRes(i) = Chi;
        curVBMInfo.ChiRes = [curVBMInfo.ChiRes, Chi];
        if Chi < ChiTH
            X = KGain * innov;
            P = P - KGain * innov_cov * KGain';
            % feedback
            ins.qnb = qdelphi(ins.qnb,  X(1:3));
            ins.vn = ins.vn - X(4:6);
            ins.pos = ins.pos -  X(7:9);
            ins.eb = ins.eb + X(10:12);
            ins.db = ins.db + X(13:15);
            ins.lever = ins.lever + X(16:18);
            ins.dly = ins.dly + X(19);
            ins.coffset = ins.coffset + X(20);
            ins.cdrift = ins.cdrift + X(21);
            [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
            ins.avp = [ins.att; ins.vn; ins.pos];
            % reset the RV
            ins.alpha = zeros(3, 1);
            curVBMInfo.ChiIndex = [curVBMInfo.ChiIndex, 1];
            curVBMInfo.time = [curVBMInfo.time, t];
        else
            Chiindex(i) = 1;
            curVBMInfo.ChiIndex = [curVBMInfo.ChiIndex,2];
            curVBMInfo.time = [curVBMInfo.time, t];
        end
        
        map(prn(i)) = curVBMInfo;
    end
    % kalman measurement update  
end

function [ins, P] = MEKFTCNHC(ins, P)
    vm = ins.Cnb'*ins.vn;
    M = ins.Cnb'*askew(ins.vn);
    C = ins.Cnb';R = 0.5^2*eye(2);
    H = [-M(1,:), C(1,:),zeros(1,13),zeros(1,2);
         -M(3,:), C(3,:),zeros(1,13),zeros(1,2)];
    

    innov_cov = H * P * H' + R;
    cross_corr_cov = P * H';
    KGain = cross_corr_cov * invbc(innov_cov);
    X = KGain * ( vm([1,3]) );
    P = P - KGain * innov_cov * KGain';
    

    ins.qnb = qdelphi(ins.qnb,  X(1:3));
    ins.vn = ins.vn - X(4:6);
    ins.pos = ins.pos -  X(7:9);
    ins.eb = ins.eb + X(10:12);
    ins.db = ins.db + X(13:15);
    ins.lever = ins.lever + X(16:18);
    ins.dly = ins.dly + X(19);
    ins.coffset = ins.coffset + X(20);
    ins.cdrift = ins.cdrift + X(21);
    [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
    ins.avp = [ins.att; ins.vn; ins.pos];
    % reset the RV
    ins.alpha = zeros(3, 1);

end

