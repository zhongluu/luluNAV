% Copyright(c) 2021, by Yulu Zhong. All rights reserved.
% Key Laboratory of Micro-Inertial Instrument and Advanced Navigation Technology of Ministry of Education,
% Southeast University, NanJing, P.R.China 10/31/2021
% based on psins toolbox from http://www.psins.org.cn/
% version:psins210406.rar
% or psins210522.rar
% Acknowledge: Gongmin Yan and Kailong Li.
close all;
clear;
glvs;

load("../EXNAVDATA_1031_1807_I.mat");
load("../EXNAVDATA_1031_1807_I_SPP.mat");


tmpposL = deg2rad(posL(1,1:2));

trj.avp0=[deg2rad([-1.178971197228000,-4.539559178293000,-8.5]),0,0,0, tmpposL,posL(1,3)];

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
% initialBiasA = 150; % m/s^2 --> ug
initialBiasA = 0.073; % m/s^2 
% GNSS err 
SIS_err_SD = 1; % Signal in space error SD (m)
zenith_iono_err_SD = 2; % Zenith ionosphere error SD (m) 
zenith_trop_err_SD = 0.2; % Zenith troposphere error SD (m) 
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
%            Markley-Crassidis. "Fundamentals Of Spacecraft Attitude Determination and Control"
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
% P = diag([(deg2rad(10/3))^2 * ones(1, 3), (0.15/3)^2 * ones(1, 2), (0.15/3)^2, 1e-6^2 * ones(1, 2), (10/3)^2, ...
%         (initialBiasG * glv.dph)^2 * ones(1, 3), (initialBiasA)^2 * ones(1, 3), rx_clock_offset^2, rx_clock_drift^2]); % same as reference
P = diag([(deg2rad(10/3))^2 * ones(1, 3), (0.15/3)^2 * ones(1, 2), (0.15/3)^2, 1e-6^2 * ones(1, 2), (10/3)^2, ...
        (initialBiasG * glv.dph)^2 * ones(1, 3), (initialBiasA)^2 * ones(1, 3), 0.8*ones(1,3), 0.005^2, rx_clock_offset^2, rx_clock_drift^2]); % same as reference

% Q =  diag([deg2rad((deviationOfGyroV) / 60)^2 * ones(1, 3), (0.8/60)^2 * ones(1, 3), 0.004^2*ones(1,3) clock_phase_PSD^2, clock_freq_PSD^2]);
Q =  diag([deg2rad((deviationOfGyroV) / 60)^2 * ones(1, 3), (0.8/60)^2 * ones(1, 3), deg2rad(0.3/3600)^2* ones(1, 3), 0.004^2*ones(1,3) clock_phase_PSD^2, clock_freq_PSD^2]);

R = [pseudo_range_SD^2, range_rate_SD^2];
seqR = pseudo_range_SD^2;
n = length(X);
% KF Init
% DKFins = insinit(avpadderr(trj.avp0,davp0), ts);
DKFins = insinit(trj.avp0', ts);
DKFins.lever = [0.1,0.1,0.8]';
DKFins.dly = 0.005;
DKFP = P;
DKFins.alpha = zeros(3,1);
% DKFins.cdrift = GNSSerr.rx_clock_drift;
% DKFins.coffset = GNSSerr.rx_clock_offset;
DKFins.cdrift = 0;
DKFins.coffset = 0;
RSTKFins = DKFins;
RSTEKFP = DKFP;
MCEKFins = RSTKFins;
MCEKFP = DKFP;
% prealloc
len = length(imu);
[RSTestRes,  MCestRes] = prealloc(fix(len / nn), length(X) + 1,length(X) + 1);
[RSTLambdaRes, MCLambdaRes] = prealloc(fix(len / nn),1,1);
timebar(nn, len, 'MineMEKFTCNV.');
ki = 1;

for k = 1:nn:len - nn + 1
    k1 = k + nn - 1;
    wvm = imu(k:k1, 1:6);
    t = imu(k1, end);

    [RSTKFins, RSTEKFP] = MEKFPredictionTC2(RSTKFins, RSTEKFP, Q, wvm, nts);
    [MCEKFins, MCEKFP] = MEKFPredictionTC2(MCEKFins, MCEKFP, Q, wvm, nts);
    %% correction
    if mod(t, TGNSS)==0
        % pick current GNSS measurement
        curGNSS.obs = GNSS.obs(:,:, ki);
        curGNSS.no_GNSS = GNSS.no_GNSS(ki, :);

        if curGNSS.no_GNSS(1) > 0
            [RSTKFins, RSTEKFP] =  MEKFTCNHC(RSTKFins, RSTEKFP); 
            [MCEKFins, MCEKFP] =  MEKFTCNHC(MCEKFins, MCEKFP); 
%             tic;
            [RSTKFins, RSTEKFP, RSTLambda, rsttime] = RSTEKFCorrectionTC(RSTKFins, RSTEKFP, seqR, curGNSS);
            RSTEKFtime(ki) = rsttime;
%             tic;
            [MCEKFins, MCEKFP, MCLambda, mctime] = MCEKFCorrectionTC(MCEKFins, MCEKFP, seqR, curGNSS);
            MCEKFTime(ki) = mctime;
        end
        % measurement updating completing

        % save filter result
        RSTestAtti = rad2deg(q2att(RSTKFins.qnb)); % deg, m/s, m
        MCestAtti = rad2deg(q2att(MCEKFins.qnb)); % deg, m/s, m
        RSTLambdaRes(ki) = RSTLambda; MCLambdaRes(ki) = MCLambda;
        RSTestRes(ki, :) = [RSTestAtti; RSTKFins.vn; RSTKFins.pos; RSTKFins.eb; RSTKFins.db; RSTKFins.lever; RSTKFins.dly; RSTKFins.coffset; RSTKFins.cdrift; t]';
        MCestRes(ki, :) = [MCestAtti; MCEKFins.vn; MCEKFins.pos; MCEKFins.eb; MCEKFins.db; MCEKFins.lever; MCEKFins.dly; MCEKFins.coffset; MCEKFins.cdrift; t]';
        ki = ki + 1;
    end

    timebar;
end
% free unnecessary space for ram
RSTestRes(ki:end,:) = []; MCestRes(ki:end,:) = []; 
RSTLambdaRes(ki:end) = []; MCLambdaRes(ki:end) = [];
%% plot
% attitude
figure('Name', 'attitude');
subplot(3, 1, 1); plot(RSTestRes(:, end), [ RSTestRes(:, 1), MCestRes(:, 1)]); hold on; % pitch
subplot(3, 1, 2); plot(RSTestRes(:, end), [ RSTestRes(:, 2), MCestRes(:, 2)]); hold on; % yaw
subplot(3, 1, 3); plot(RSTestRes(:, end), [ RSTestRes(:, 3), MCestRes(:, 3)]); hold off; % roll
legend('Estimation');
% velocity
figure('Name', 'velocity');
subplot(3, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 4), MCestRes(:, 4)]); hold on; % east
subplot(3, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 5), MCestRes(:, 5)]); hold on; % north
subplot(3, 1, 3); plot(RSTestRes(:, end), [RSTestRes(:, 6), MCestRes(:, 6)]); hold on; % up
legend('Estimation');
% position
figure('Name', 'position');
subplot(3, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 7), MCestRes(:, 7)]); hold on; % latitude
subplot(3, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 8), MCestRes(:, 8)]); hold on; % longitude
subplot(3, 1, 3); plot(RSTestRes(:, end), [RSTestRes(:, 9), MCestRes(:, 9)]); hold on; % height
legend('Estimation');
% gyro bias estimation
figure('Name', 'gyroscope bias');
subplot(3, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 10), MCestRes(:, 10)]); hold on; % x
subplot(3, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 11), MCestRes(:, 11)]); hold on; % y
subplot(3, 1, 3); plot(RSTestRes(:, end), [RSTestRes(:, 12), MCestRes(:, 12)]); hold on; % z
legend('Estimation');
% acc bias estimation
figure('Name', 'accelerate bias');
subplot(3, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 13), MCestRes(:, 13)]); hold on; % x
subplot(3, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 14), MCestRes(:, 14)]); hold on; % y
subplot(3, 1, 3); plot(RSTestRes(:, end), [RSTestRes(:, 15), MCestRes(:, 15)]); hold on; % z
legend('Estimation');
% lever estimation
figure('Name', 'lever bias');
subplot(3, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 16), MCestRes(:, 16)]); hold on; % x
subplot(3, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 17), MCestRes(:, 17)]); hold on; % y
subplot(3, 1, 3); plot(RSTestRes(:, end), [RSTestRes(:, 18), MCestRes(:, 18)]); hold on; % z
legend('Estimation');
% dly estimation
figure('Name', 'dly bias');
plot(RSTestRes(:, end), [RSTestRes(:, 19), MCestRes(:, 19)]); hold on; % x
legend('Estimation');
% clock offset and clock drift
figure('Name', 'clock offset and clock drift');
subplot(2, 1, 1); plot(RSTestRes(:, end), [RSTestRes(:, 20), MCestRes(:, 20)]); hold on; % x
subplot(2, 1, 2); plot(RSTestRes(:, end), [RSTestRes(:, 21), MCestRes(:, 21)]); hold on; % y
legend('True','Estimation');
% trajectory
posL(:,1:2) = deg2rad(posL(:,1:2));
delta = posL(1,:) - RSTestRes(1,7:9);
posL(:,1) = posL(:,1) - delta(1);posL(:,2) = posL(:,2) - delta(2);
posL(:,3) = posL(:,3) - delta(3);

% delta(1) = RTK.lat(1) - seqestRes(1,7);
% delta(2) = RTK.lon(1) - seqestRes(1,8);
% delta(3) = RTK.alt(1) - seqestRes(1,9);
% RTK.lat = RTK.lat - delta(1);
% RTK.lon = RTK.lon - delta(2);
% RTK.alt = RTK.alt - delta(3);

deg = fix(refPOS(:,1:2)/100);
minsec = refPOS(:,1:2) - deg*100;
 
refPOS(:,1:2) = deg2rad(deg + minsec / 60);

delta = refPOS(1,:) - RSTestRes(1,7:9);
refPOS(:,1) = refPOS(:,1) - delta(1);refPOS(:,2) = refPOS(:,2) - delta(2);
refPOS(:,3) = refPOS(:,3) - delta(3);
% tcINSLog = tcINSSIMLog0123;
figure('Name', 'vehicle tracjectory');
plot3([RSTestRes(:, 7)],[RSTestRes(:, 8)],RSTestRes(:, 9));hold on;
plot3([RSTestRes(1, 7)],[RSTestRes(1, 8)],RSTestRes(1, 9),'^');hold on;
plot3([MCestRes(:, 7)],[MCestRes(:, 8)],MCestRes(:, 9));hold on;
plot3([MCestRes(1, 7)],[MCestRes(1, 8)],MCestRes(1, 9),'^');hold on;
plot3(posL(:,1),posL(:,2),posL(:,3),'*');
plot3([refPOS(:, 1)],[refPOS(:, 2)],refPOS(:, 3));hold on;
% plot3(RTK.lat,RTK.lon,RTK.alt);hold on;
legend('RSTTC','RSTstart','MCTC','MCstart','spp','gnss');
figure;plot(RSTestRes(:, end),[RSTLambdaRes,MCLambdaRes]);

% MEKF filter basic function

function [ins, P] = MEKFPredictionTC2(ins, P, Q, wvm, nts)
    % neccessary parameters
    glvs;
    ins = insupdate(ins, wvm);
    ins.coffset = ins.coffset + ins.cdrift * nts;
%     [Fhi,G] = MineFtTC(ins);
    Fhi= etm(ins);
    Fhi = [Fhi, zeros(15,6);
            zeros(4,21);
            zeros(1,20),1;
            zeros(1,21)];
%     Fhi = expm(Fhi * nts);
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

function [ins, P, lambda, time] = RSTEKFCorrectionTC(ins, P, R, GNSS)
    lightspeed = 299792458; % Speed of light in m/s
    % get state nums and valid GNSS nums
    [stateNum, ~] = size(P);
    no_GNSS = GNSS.no_GNSS;
    GNSS = GNSS.obs;
    staDly = GNSS(1:no_GNSS(1),5);
    lonoDly = GNSS(1:no_GNSS(1),6);
    tropDly = GNSS(1:no_GNSS(1),7);
    % get est vel and pos from est ins   
    ins = inslever(ins);
    ins.posL = ins.posL-ins.Mpvvn * ins.dly;
    parXYZBLH = Dblh2Dxyz(ins.posL);
    Pn = ins.posL;
    [Pe, ~] = blh2xyz(Pn); r_ea_e = Pe;
    obs_meas = zeros(no_GNSS(1), 1);
    H = [];  % the H matrix is defined as dynamic due to the observed satilles number is unknow in real scenario
    rV = [];
    % RST params
    maxIter = 100; nu = 3; lambda = 1; errTH = 1e-5;
    tic;
    for i = 1 : no_GNSS(1)
    % cal est obs value range and range rate and H matrix  
        sat_r_es_e = GNSS(i, 2:4); 
        delta_r = sat_r_es_e' - r_ea_e;
        approx_range = sqrt(delta_r' * delta_r);
        u_as_e = delta_r / approx_range;
        Omega_ie = askew([0,0,ins.eth.wie]);
        C_e_I = (eye(3) - Omega_ie * (approx_range / lightspeed));
        delta_r = C_e_I * sat_r_es_e' - r_ea_e;
        obs_meas(i) = sqrt(delta_r' * delta_r) + ins.coffset; % for range
        H = [H; zeros(1,3), zeros(1,3), u_as_e'*parXYZBLH,  zeros(1,6),...
            -u_as_e'*parXYZBLH*ins.MpvCnb, u_as_e'*parXYZBLH*ins.Mpvvn  1, 0];
        rV = [rV;(GNSS(i,1)+staDly(i) +lonoDly(i) +  tropDly(i))];
    end
    yt = zeros(no_GNSS(1),1); X = zeros(stateNum,1);tmpP = P;
    for i = 1:maxIter
        preX = X;yt = [];
        Rt = R*diag(ones(1,no_GNSS(1))) / lambda;
        innov_cov = H * tmpP * H' + Rt;
        KGain = tmpP * H' * invbc(innov_cov);
        X = KGain * (rV - obs_meas);
        P = tmpP - KGain * innov_cov * KGain';
        %
        tmpPos = ins.posL -  X(7:9) + ins.MpvCnb*X(16:18) - ins.Mpvvn *X(19);
        [r_ea_e, ~] = blh2xyz(tmpPos);
        for j = 1:no_GNSS(1)
            
            sat_r_es_e = GNSS(j, 2:4); 
            delta_r = sat_r_es_e' - r_ea_e;
            approx_range = sqrt(delta_r' * delta_r);
            C_e_I = (eye(3) - Omega_ie * (approx_range / lightspeed));
            delta_r = C_e_I * sat_r_es_e' - r_ea_e;
            yt = [yt; (sqrt(delta_r' * delta_r) + ins.coffset + X(20))];
        end
        tmpinnov2 = (rV - yt)*(rV - yt)';
        tmpHPH =  H * P * H';
        Bt = tmpinnov2 + tmpHPH;
        tbr = Bt * invbc(R*diag(ones(1,no_GNSS(1))));
        lambda = (nu + stateNum)/(nu + trace(tbr));
        if lambda<1e-6, lambda = 1e-6;  end
        err = norm((X - preX));
        if err < errTH
            break;
        end
    end
    
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
    % kalman measurement update  
    time = toc;
end

function [ins, P, lambda, time] = MCEKFCorrectionTC(ins, P, R, GNSS)
    lightspeed = 299792458; % Speed of light in m/s
    % get state nums and valid GNSS nums
    [stateNum, ~] = size(P);
    no_GNSS = GNSS.no_GNSS;
    GNSS = GNSS.obs;
    staDly = GNSS(1:no_GNSS(1),5);
    lonoDly = GNSS(1:no_GNSS(1),6);
    tropDly = GNSS(1:no_GNSS(1),7);
    % get est vel and pos from est ins   
    ins = inslever(ins);
    ins.posL = ins.posL-ins.Mpvvn * ins.dly;
    parXYZBLH = Dblh2Dxyz(ins.posL);
    Pn = ins.posL;
    [Pe, ~] = blh2xyz(Pn); r_ea_e = Pe;
    obs_meas = zeros(no_GNSS(1), 1);
    H = [];  % the H matrix is defined as dynamic due to the observed satilles number is unknow in real scenario
    rV = [];
    % MC params
    maxIter = 100; sigma = 5; lambda = 1; errTH = 1e-5;
    tic;
    for i = 1 : no_GNSS(1)
    % cal est obs value range and range rate and H matrix  
        sat_r_es_e = GNSS(i, 2:4); 
        delta_r = sat_r_es_e' - r_ea_e;
        approx_range = sqrt(delta_r' * delta_r);
        u_as_e = delta_r / approx_range;
        Omega_ie = askew([0,0,ins.eth.wie]);
        C_e_I = (eye(3) - Omega_ie * (approx_range / lightspeed));
        delta_r = C_e_I * sat_r_es_e' - r_ea_e;
        obs_meas(i) = sqrt(delta_r' * delta_r) + ins.coffset; % for range
        H = [H; zeros(1,3), zeros(1,3), u_as_e'*parXYZBLH,  zeros(1,6),...
            -u_as_e'*parXYZBLH*ins.MpvCnb, u_as_e'*parXYZBLH*ins.Mpvvn  1, 0];
        rV = [rV;(GNSS(i,1)+staDly(i) +lonoDly(i) +  tropDly(i))];
    end
    X = zeros(stateNum,1);tmpP = P;%yt = zeros(no_GNSS(1),1); 
    for i = 1:maxIter
        preX = X; yt = [];
        Rt = R*diag(ones(1,no_GNSS(1))) / lambda;
        innov_cov = H * tmpP * H' + Rt;
        KGain = tmpP * H'  * invbc(innov_cov);
        X = KGain * (rV - obs_meas);
        P = tmpP - KGain * innov_cov * KGain';
        %
        tmpPos = ins.posL -  X(7:9) + ins.MpvCnb*X(16:18) - ins.Mpvvn *X(19);
        [r_ea_e, ~] = blh2xyz(tmpPos);
        for j = 1:no_GNSS(1)
            sat_r_es_e = GNSS(j, 2:4); 
            delta_r = sat_r_es_e' - r_ea_e;
            approx_range = sqrt(delta_r' * delta_r);
            C_e_I = (eye(3) - Omega_ie * (approx_range / lightspeed));
            delta_r = C_e_I * sat_r_es_e' - r_ea_e;
            yt = [yt; (sqrt(delta_r' * delta_r) + ins.coffset + X(20))];
        end
        tmpinnov2 = (rV - yt)*(rV - yt)';
        tmpHPH =  H * P * H';
        Bt = tmpinnov2 + tmpHPH;
        tbr = Bt * invbc(R*diag(ones(1,no_GNSS(1))));
        lambda = exp((1-trace(tbr))/2/sigma^2);
        if lambda<1e-6, lambda = 1e-6;  end
        err = norm((X - preX));
        if err < errTH
            break;
        end
    end
    
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
    % kalman measurement update  
    time = toc;
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

