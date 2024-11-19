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

%% Inv-Gamma 5 dimension Bearing only tracking nonlinear model
% Yulu Zhong
% 5 state variables u, dot{u}, v, dot{v}, omega
% nonlinear observer
% 2024/3/27 southeast university
% [1] H. Wang, H. Li, J. Fang, and H. Wang, “Robust Gaussian Kalman Filter With Outlier Detection,” IEEE SIGNAL PROCESSING LETTERS, vol. 25, no. 8, pp. 1236–1240, Aug. 2018, doi: 10.1109/LSP.2018.2851156.
% [2] S. Sarkka, "Variational Bayesian Adaptation of Noise Covariances in Non-Linear Kalman Filtering,” arXiv, 2013.
% [3] S. Sarkka and A. Nummenmaa, "Recursive Noise Adaptive Kalman Filtering by Variational Bayesian Approximations,” IEEE T Automat Contr, vol. 54, no. 3, pp. 596–600, Mar. 2009, doi: 10.1109/tac.2008.2008348.

%% Begin
clear;
close all;
clc;
% global parameters set
deltaT = 1;
stopT = 150;
varyingBegin = 20;
varyingEnd = 30;
outlierBegin = 100;
outlierEnd = 105;
susoutlierBegin = 120;
susoutlierEnd = 140;
MC_times = 100;
VB_D_Err_TH = 1e-100;
VB_it_Max = 100;

Time = zeros(1, fix(stopT/deltaT));

RMSE_CHIEKF_X = zeros(MC_times,5); % prealloc MC result for EKF
RMSE_VBEKF_X = zeros(MC_times,5); % prealloc MC result for VBEKF
RMSE_VBEKF_R = zeros(MC_times,4); % prealloc MC result for VBEKF
RMSE_BIWVBEKF_X = zeros(MC_times,5); % prealloc MC result for BIWVBEKF
RMSE_BIWVBEKF_R = zeros(MC_times,4); % prealloc MC result for BIWVBEKF
RMSE_BIWVBEKFSEQ_X = zeros(MC_times,5); % prealloc MC result for BIWVBEKF
RMSE_BIWVBEKFSEQ_R = zeros(MC_times,4); % prealloc MC result for BIWVBEKF

% model parameters set
SensorsPoint = [ 5e2,  5e2;
                -5e2,  5e2;
                 5e2, -5e2;
                -5e2, -5e2];

q1 = 0.1; q2 = 1.75e-6;
M = [deltaT^3/3, deltaT^2/2;
     deltaT^2/2, deltaT;    ];
Q = [q1*M,       zeros(2,2), zeros(2,1);
     zeros(2,2), q1*M,       zeros(2,1);
     zeros(1,2), zeros(1,2),          q2;];

nominal_R = 25^2 .* eye(4);
abnomal_Lambda = 0.5;
abnomal_Alpha = 5000;
sustainMu = 5000;

X0 = [-0, 5, 0, 5, 0.1]';

R = 25^2 .* ones(4, fix(stopT/deltaT)) - 20^2.* cos(0.02*(0:1:fix(stopT/deltaT)-1));

outlierGm_Mu = zeros(2,4);


% true model 
Xpse = zeros(5, fix(stopT/deltaT)); % prealloc true X
Ypse = zeros(4, fix(stopT/deltaT)); % prealloc true Y

for mc = 1:MC_times
%% model stimulate
    Xpse(:,1) = X0;
    k = 1;
    for t = 0 : deltaT : stopT-deltaT
        Xpse(:,k+1) = Ft(Xpse(:,k), deltaT) + mvnrnd(zeros(5,1), Q)';
        if (t > outlierBegin)&&(t <outlierEnd)
            outlierGm_Sigma = zeros(4,4,2);
            outlierGm_Sigma(:,:,1) = diag(R(:,k));
            abnomal_R = abnomal_Alpha * diag(R(:,k));
            outlierGm_Sigma(:,:,2) = abnomal_R;
            outlierGm_P = [1-abnomal_Lambda,abnomal_Lambda];
            outlierGm = gmdistribution(outlierGm_Mu, outlierGm_Sigma, outlierGm_P);
            Ypse(:,k) = Ht(Xpse(:,k+1), SensorsPoint)+ random(outlierGm,1)';
        elseif (t > susoutlierBegin)&&(t <susoutlierEnd)
            Ypse(:,k) = Ht(Xpse(:,k+1), SensorsPoint)+ mvnrnd(sustainMu*ones(4,1), diag(R(:,k)))';
        else
            Ypse(:,k) = Ht(Xpse(:,k+1), SensorsPoint)+ mvnrnd(zeros(4,1), diag(R(:,k)))';
        end
        Time(k) = t;
        k = k+1;
    end
    filter_P0 = 1*Q;
    filter_X0 = X0 + chol(filter_P0)*randn(5,1);

%% EKF
    resChiEKF_X = zeros(5, fix(stopT/deltaT));
    resSEQEKFCHI = zeros(4, fix(stopT/deltaT));
    resSEQEKFCHIIndex = zeros(4, fix(stopT/deltaT));
    ChiEKF_Px = filter_P0;
    ChiEKF_X = filter_X0;
    EKF_R = nominal_R;
%     EKF_R = diag(R(:,1));
    SEQEKFCHIth = 4000;
    resEKF_RMSE = zeros(1, fix(stopT/deltaT));
    k=1;
    for t = 0 : deltaT : stopT-deltaT
        % time update
        F = JabF(ChiEKF_X, deltaT);
        ChiEKF_X = Ft(ChiEKF_X, deltaT);
        ChiEKF_Px = F*ChiEKF_Px*F'+ Q;
        % measurement update
        H = JabH(ChiEKF_X, SensorsPoint);
        EKF_Hk = H;
        SEQEKFHn = size(EKF_Hk,1);
        for i = 1:SEQEKFHn
            EKF_Sk = EKF_Hk(i,:)*ChiEKF_Px*EKF_Hk(i,:)'+ EKF_R(i,i);
            SEQEKFCHI = (Ypse(i,k) - EKF_Hk(i,:)*ChiEKF_X)'/EKF_Sk*(Ypse(i,k) - EKF_Hk(i,:)*ChiEKF_X);
            resSEQEKFCHI(i,k) = SEQEKFCHI;
            if (SEQEKFCHI < SEQEKFCHIth)
                resSEQEKFCHIIndex(i,k) = 1;
                EKF_Kk = ChiEKF_Px*EKF_Hk(i,:)'/EKF_Sk;
                ChiEKF_X = ChiEKF_X + EKF_Kk*(Ypse(i,k) - Ht(ChiEKF_X, SensorsPoint(i,:)));
                ChiEKF_Px = ChiEKF_Px - EKF_Kk*EKF_Sk*EKF_Kk';
            end
        end
        % res
        resChiEKF_X(:,k) = ChiEKF_X;
        resEKF_RMSE(:,k) = sqrt(trace(ChiEKF_Px));
        k=k+1;
    end

%% VBEKF
    resVBEKF_X_d = zeros(5, fix(stopT/deltaT));
    resVBEKF_R_d = zeros(4, fix(stopT/deltaT));
    resVBEKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resVBEKF_it_d = zeros(1, fix(stopT/deltaT));
    VBEKF_X_d = filter_X0;
    VBEKF_Px_d = filter_P0;  
    VBEKF_IGa1_d = 1; VBEKF_IGb1_d = R(1,1);%VBEKF_IGb1_d = 50^2; % Inv-Gamma param.
    VBEKF_IGa2_d = 1; VBEKF_IGb2_d = R(2,1);%VBEKF_IGb2_d = 50^2; % Inv-Gamma param.
    VBEKF_IGa3_d = 1; VBEKF_IGb3_d = R(3,1);%VBEKF_IGb3_d = 50^2; % Inv-Gamma param.
    VBEKF_IGa4_d = 1; VBEKF_IGb4_d = R(4,1);%VBEKF_IGb4_d = 50^2; % Inv-Gamma param.
%     VBEKF_it_max_d = 20; % iteration times
    VBEKF_it_max_d = VB_it_Max;
    VBEKF_E_d = VB_D_Err_TH; % error threshold
    VBEKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        % time update
        F = JabF(VBEKF_X_d, deltaT);
        VBEKF_X_d = Ft(VBEKF_X_d, deltaT);
        VBEKF_Px_d = F*VBEKF_Px_d*F'+ Q;
        VBEKF_IGa1_d = VBEKF_pho_d * VBEKF_IGa1_d; VBEKF_IGa2_d = VBEKF_pho_d * VBEKF_IGa2_d; % heuristic dynamic
        VBEKF_IGb1_d = VBEKF_pho_d * VBEKF_IGb1_d; VBEKF_IGb2_d = VBEKF_pho_d * VBEKF_IGb2_d; % heuristic dynamic
        VBEKF_IGa3_d = VBEKF_pho_d * VBEKF_IGa3_d; VBEKF_IGa4_d = VBEKF_pho_d * VBEKF_IGa4_d; % heuristic dynamic
        VBEKF_IGb3_d = VBEKF_pho_d * VBEKF_IGb3_d; VBEKF_IGb4_d = VBEKF_pho_d * VBEKF_IGb4_d; % heuristic dynamic
        % measurement update
        H = JabH(VBEKF_X_d, SensorsPoint);
        VBEKF_Hk_d = H;
        VBEKF_IGa1_d = 0.5 + VBEKF_IGa1_d; % VB step1
        VBEKF_IGa2_d = 0.5 + VBEKF_IGa2_d; 
        VBEKF_IGa3_d = 0.5 + VBEKF_IGa3_d; 
        VBEKF_IGa4_d = 0.5 + VBEKF_IGa4_d; 
        tmpVBEKF_X_d = VBEKF_X_d; tmpVBEKF_IGb1_d = VBEKF_IGb1_d; 
        tmpVBEKF_Px_d = VBEKF_Px_d; tmpVBEKF_IGb2_d = VBEKF_IGb2_d;
        tmpVBEKF_IGb3_d = VBEKF_IGb3_d;tmpVBEKF_IGb4_d = VBEKF_IGb4_d;
        for i = 1 : VBEKF_it_max_d % VB iterate dynamic times
            VBEKF_X_d_pre = VBEKF_X_d;
            VBEKF_R_d = diag([VBEKF_IGb1_d / VBEKF_IGa1_d, VBEKF_IGb2_d / VBEKF_IGa2_d,...
                VBEKF_IGb3_d / VBEKF_IGa3_d, VBEKF_IGb4_d / VBEKF_IGa4_d]); % VB-E step begin
            VBEKF_Sk_d = VBEKF_Hk_d * tmpVBEKF_Px_d * VBEKF_Hk_d'+ VBEKF_R_d;
            VBEKF_Kk_d = tmpVBEKF_Px_d * VBEKF_Hk_d' / VBEKF_Sk_d;
            VBEKF_X_d = tmpVBEKF_X_d + VBEKF_Kk_d * (Ypse(:,k) - Ht(tmpVBEKF_X_d, SensorsPoint));
            VBEKF_Px_d = tmpVBEKF_Px_d - VBEKF_Kk_d * VBEKF_Sk_d * VBEKF_Kk_d'; % VB-E step done
            tmp = [tmpVBEKF_IGb1_d; tmpVBEKF_IGb2_d; tmpVBEKF_IGb3_d; tmpVBEKF_IGb4_d] + ... % VB-M step begin
                0.5*(Ypse(:,k) - Ht(VBEKF_X_d, SensorsPoint)).^2 + ...
                0.5*diag((VBEKF_Hk_d * VBEKF_Px_d * VBEKF_Hk_d')); 
            VBEKF_IGb1_d = tmp(1); VBEKF_IGb2_d = tmp(2);
            VBEKF_IGb3_d = tmp(3); VBEKF_IGb4_d = tmp(4);% VB-M step done        
            if(norm(VBEKF_X_d-VBEKF_X_d_pre) < VBEKF_E_d)
                break;
            end
        end
        resVBEKF_it_d(:,k) = i;
        %res
        resVBEKF_X_d(:,k) = VBEKF_X_d;
        resVBEKF_R_d(:,k) = diag(VBEKF_R_d);
        resVBEKF_RMSE_d(:,k) = sqrt(trace(VBEKF_Px_d));
        k=k+1;
    end   

%% BIWVBEKF (bern inv Gamma) - MINE
    resBIWVBEKF_X_d = zeros(5, fix(stopT/deltaT));
    resBIWVBEKF_R_d = zeros(4, fix(stopT/deltaT));
    resBIWVBEKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resBIWVBEKF_it_d = zeros(1, fix(stopT/deltaT));
    resBIWVBZt = zeros(4, fix(stopT/deltaT));
    BIWVBEKF_X_d = filter_X0;
    BIWVBEKF_Px_d = filter_P0;  
    BIWVBEKF_BerZt_d = 1; % Bernoulli param. 
    BIWVBEKF_BerZt_TH = 0.1;
    BIWVBEKF_BetaE0_d = 0.9; BIWVBEKF_BetaF0_d = 0.1; % Beta params.
%     BIWVBEKF_BerZt_TH = 0.1;
%     BIWVBEKF_BetaE0_d = 0.8; BIWVBEKF_BetaF0_d = 0.2; % Beta params.
    BIWVBEKF_IGa1_d = 1; BIWVBEKF_IGb1_d = R(1,1);%BIWVBEKF_IGb1_d = 50^2; % Inv-Gamma param.
    BIWVBEKF_IGa2_d = 1; BIWVBEKF_IGb2_d = R(2,1);%BIWVBEKF_IGb2_d = 50^2; % Inv-Gamma param.
    BIWVBEKF_IGa3_d = 1; BIWVBEKF_IGb3_d = R(3,1);%BIWVBEKF_IGb3_d = 50^2; % Inv-Gamma param.
    BIWVBEKF_IGa4_d = 1; BIWVBEKF_IGb4_d = R(4,1);%BIWVBEKF_IGb4_d = 50^2; % Inv-Gamma param.
%     BIWVBEKF_it_max_d = 20; % iteration times
    BIWVBEKF_it_max_d = VB_it_Max;
    BIWVBEKF_E_d = VB_D_Err_TH; % error threshold
    BIWVBEKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0 : deltaT : stopT - deltaT
        % time update
        F = JabF(BIWVBEKF_X_d, deltaT); 
        BIWVBEKF_X_d = Ft(BIWVBEKF_X_d, deltaT); 
        BIWVBEKF_Px_d = F * BIWVBEKF_Px_d * F' + Q; 
        BIWVBEKF_IGa1_d = BIWVBEKF_pho_d * BIWVBEKF_IGa1_d; BIWVBEKF_IGa2_d = BIWVBEKF_pho_d * BIWVBEKF_IGa2_d; % heuristic dynamic
        BIWVBEKF_IGb1_d = BIWVBEKF_pho_d * BIWVBEKF_IGb1_d; BIWVBEKF_IGb2_d = BIWVBEKF_pho_d * BIWVBEKF_IGb2_d; % heuristic dynamic
        BIWVBEKF_IGa3_d = BIWVBEKF_pho_d * BIWVBEKF_IGa3_d; BIWVBEKF_IGa4_d = BIWVBEKF_pho_d * BIWVBEKF_IGa4_d; % heuristic dynamic
        BIWVBEKF_IGb3_d = BIWVBEKF_pho_d * BIWVBEKF_IGb3_d; BIWVBEKF_IGb4_d = BIWVBEKF_pho_d * BIWVBEKF_IGb4_d; % heuristic dynamic
        % measurement update
        H = JabH(BIWVBEKF_X_d, SensorsPoint);
%         BIWVBEKF_Hk_d = H;
        BIWVBEKF_IGa1_d = 0.5 + BIWVBEKF_IGa1_d; % VB step1
        BIWVBEKF_IGa2_d = 0.5 + BIWVBEKF_IGa2_d; 
        BIWVBEKF_IGa3_d = 0.5 + BIWVBEKF_IGa3_d; 
        BIWVBEKF_IGa4_d = 0.5 + BIWVBEKF_IGa4_d; 
        tmpBIWVBEKF_BerZt1_d = BIWVBEKF_BerZt_d; 
        tmpBIWVBEKF_BerZt2_d = BIWVBEKF_BerZt_d;
        tmpBIWVBEKF_BerZt3_d = BIWVBEKF_BerZt_d;
        tmpBIWVBEKF_BerZt4_d = BIWVBEKF_BerZt_d;
        BIWVBEKF_BetaEt1_d = BIWVBEKF_BetaE0_d; BIWVBEKF_BetaFt1_d = BIWVBEKF_BetaF0_d;
        BIWVBEKF_BetaEt2_d = BIWVBEKF_BetaE0_d; BIWVBEKF_BetaFt2_d = BIWVBEKF_BetaF0_d;
        BIWVBEKF_BetaEt3_d = BIWVBEKF_BetaE0_d; BIWVBEKF_BetaFt3_d = BIWVBEKF_BetaF0_d;
        BIWVBEKF_BetaEt4_d = BIWVBEKF_BetaE0_d; BIWVBEKF_BetaFt4_d = BIWVBEKF_BetaF0_d;
        tmpBIWVBEKF_X_d = BIWVBEKF_X_d; tmpBIWVBEKF_IGb1_d = BIWVBEKF_IGb1_d; 
        tmpBIWVBEKF_Px_d = BIWVBEKF_Px_d; tmpBIWVBEKF_IGb2_d = BIWVBEKF_IGb2_d;
        tmpBIWVBEKF_IGb3_d = BIWVBEKF_IGb3_d; tmpBIWVBEKF_IGb4_d = BIWVBEKF_IGb4_d;
        for i = 1 : BIWVBEKF_it_max_d % VB iterate dynamic times
%             H = JabH(BIWVBEKF_X_d, SensorsPoint);
            BIWVBEKF_X_d_pre = BIWVBEKF_X_d;
            if ((tmpBIWVBEKF_BerZt1_d > BIWVBEKF_BerZt_TH) || (tmpBIWVBEKF_BerZt2_d > BIWVBEKF_BerZt_TH) || (tmpBIWVBEKF_BerZt3_d > BIWVBEKF_BerZt_TH) || (tmpBIWVBEKF_BerZt4_d > BIWVBEKF_BerZt_TH))
                BIWVBEKF_Hk_d = []; BIWVBEKF_R_d = []; BIWVBEKFSensorsPoint = []; YBIWVBEKF = [];
                if tmpBIWVBEKF_BerZt1_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Hk_d = [BIWVBEKF_Hk_d; H(1,:)]; 
                    YBIWVBEKF = [YBIWVBEKF; Ypse(1,k)]; 
                    BIWVBEKFSensorsPoint = [BIWVBEKFSensorsPoint; SensorsPoint(1,:)]; 
                    BIWVBEKF_R_d = [BIWVBEKF_R_d, BIWVBEKF_IGb1_d / BIWVBEKF_IGa1_d / tmpBIWVBEKF_BerZt1_d]; 
                end
                if tmpBIWVBEKF_BerZt2_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Hk_d = [BIWVBEKF_Hk_d; H(2,:)]; 
                    YBIWVBEKF = [YBIWVBEKF; Ypse(2,k)]; 
                    BIWVBEKFSensorsPoint = [BIWVBEKFSensorsPoint; SensorsPoint(2,:)]; 
                    BIWVBEKF_R_d = [BIWVBEKF_R_d, BIWVBEKF_IGb2_d / BIWVBEKF_IGa2_d / tmpBIWVBEKF_BerZt2_d]; 
                end
                if tmpBIWVBEKF_BerZt3_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Hk_d = [BIWVBEKF_Hk_d; H(3,:)];
                    YBIWVBEKF = [YBIWVBEKF; Ypse(3,k)];
                    BIWVBEKFSensorsPoint = [BIWVBEKFSensorsPoint; SensorsPoint(3,:)];
                    BIWVBEKF_R_d = [BIWVBEKF_R_d, BIWVBEKF_IGb3_d / BIWVBEKF_IGa3_d / tmpBIWVBEKF_BerZt3_d];
                end
                if tmpBIWVBEKF_BerZt4_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Hk_d = [BIWVBEKF_Hk_d; H(4,:)];
                    YBIWVBEKF = [YBIWVBEKF; Ypse(4,k)];
                    BIWVBEKFSensorsPoint = [BIWVBEKFSensorsPoint; SensorsPoint(4,:)];
                    BIWVBEKF_R_d = [BIWVBEKF_R_d, BIWVBEKF_IGb4_d / BIWVBEKF_IGa4_d / tmpBIWVBEKF_BerZt4_d];
                end
                BIWVBEKF_R_d = diag(BIWVBEKF_R_d);
                BIWVBEKF_Sk_d = BIWVBEKF_Hk_d * tmpBIWVBEKF_Px_d * BIWVBEKF_Hk_d'+ BIWVBEKF_R_d; 
                BIWVBEKF_Kk_d = tmpBIWVBEKF_Px_d * BIWVBEKF_Hk_d' / BIWVBEKF_Sk_d; 
                BIWVBEKF_X_d = tmpBIWVBEKF_X_d + BIWVBEKF_Kk_d * (YBIWVBEKF - Ht(tmpBIWVBEKF_X_d, BIWVBEKFSensorsPoint)); 
                BIWVBEKF_Px_d = tmpBIWVBEKF_Px_d - BIWVBEKF_Kk_d * BIWVBEKF_Sk_d * BIWVBEKF_Kk_d'; % VB-E step done 
                if tmpBIWVBEKF_BerZt1_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Bt = (Ypse(1,k) - Ht(BIWVBEKF_X_d, SensorsPoint(1,:))) * (Ypse(1,k) - Ht(BIWVBEKF_X_d, SensorsPoint(1,:)))' +  H(1,:) * BIWVBEKF_Px_d * H(1,:)';
                    tmpBIWVBEKF_BerZt1_d = exp(psi(BIWVBEKF_BetaEt1_d) - psi(BIWVBEKF_BetaEt1_d + BIWVBEKF_BetaFt1_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa1_d / BIWVBEKF_IGb1_d)) / ...
                             (exp(psi(BIWVBEKF_BetaEt1_d) - psi(BIWVBEKF_BetaEt1_d + BIWVBEKF_BetaFt1_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa1_d / BIWVBEKF_IGb1_d)) + ...
                                exp(psi(BIWVBEKF_BetaFt1_d ) - psi(BIWVBEKF_BetaEt1_d + BIWVBEKF_BetaFt1_d )));
                    BIWVBEKF_IGb1_d = tmpBIWVBEKF_IGb1_d +  0.5 * tmpBIWVBEKF_BerZt1_d * (Ypse(1,k) - Ht(BIWVBEKF_X_d,  SensorsPoint(1,:))).^2 + ...
                        0.5 * tmpBIWVBEKF_BerZt1_d * diag((H(1,:) * BIWVBEKF_Px_d * H(1,:)')); 
                    BIWVBEKF_BetaEt1_d = BIWVBEKF_BetaE0_d + tmpBIWVBEKF_BerZt1_d; 
                    BIWVBEKF_BetaFt1_d = BIWVBEKF_BetaF0_d + 1 - tmpBIWVBEKF_BerZt1_d; % VB-M step done 
                else
                    BIWVBEKF_IGb1_d = tmpBIWVBEKF_IGb1_d; 
                end
                if tmpBIWVBEKF_BerZt2_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Bt = (Ypse(2,k) - Ht(BIWVBEKF_X_d, SensorsPoint(2,:))) * (Ypse(2,k) - Ht(BIWVBEKF_X_d, SensorsPoint(2,:)))' +  H(2,:) * BIWVBEKF_Px_d * H(2,:)';
                    tmpBIWVBEKF_BerZt2_d = exp(psi(BIWVBEKF_BetaEt2_d) - psi(BIWVBEKF_BetaEt2_d + BIWVBEKF_BetaFt2_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa2_d / BIWVBEKF_IGb2_d)) / ...
                             (exp(psi(BIWVBEKF_BetaEt2_d) - psi(BIWVBEKF_BetaEt2_d + BIWVBEKF_BetaFt2_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa2_d / BIWVBEKF_IGb2_d)) + ...
                                exp(psi(BIWVBEKF_BetaFt2_d ) - psi(BIWVBEKF_BetaEt2_d + BIWVBEKF_BetaFt2_d )));
                    BIWVBEKF_IGb2_d = tmpBIWVBEKF_IGb2_d +  0.5 * tmpBIWVBEKF_BerZt2_d * (Ypse(2,k) - Ht(BIWVBEKF_X_d,  SensorsPoint(2,:))).^2 + ...
                        0.5 * tmpBIWVBEKF_BerZt2_d * diag((H(2,:) * BIWVBEKF_Px_d * H(2,:)')); 
                    BIWVBEKF_BetaEt2_d = BIWVBEKF_BetaE0_d + tmpBIWVBEKF_BerZt2_d;
                    BIWVBEKF_BetaFt2_d = BIWVBEKF_BetaF0_d + 1 - tmpBIWVBEKF_BerZt2_d; % VB-M step done 
                else
                    BIWVBEKF_IGb2_d = tmpBIWVBEKF_IGb2_d; 
                end
                if tmpBIWVBEKF_BerZt3_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Bt = (Ypse(3,k) - Ht(BIWVBEKF_X_d, SensorsPoint(3,:))) * (Ypse(3,k) - Ht(BIWVBEKF_X_d, SensorsPoint(3,:)))' +  H(3,:) * BIWVBEKF_Px_d * H(3,:)';
                    tmpBIWVBEKF_BerZt3_d = exp(psi(BIWVBEKF_BetaEt3_d) - psi(BIWVBEKF_BetaEt3_d + BIWVBEKF_BetaFt3_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa3_d / BIWVBEKF_IGb3_d)) / ...
                             (exp(psi(BIWVBEKF_BetaEt3_d) - psi(BIWVBEKF_BetaEt3_d + BIWVBEKF_BetaFt3_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa3_d / BIWVBEKF_IGb3_d)) + ...
                                exp(psi(BIWVBEKF_BetaFt3_d ) - psi(BIWVBEKF_BetaEt3_d + BIWVBEKF_BetaFt3_d )));
                    BIWVBEKF_IGb3_d = tmpBIWVBEKF_IGb3_d +  0.5 * tmpBIWVBEKF_BerZt3_d * (Ypse(3,k) - Ht(BIWVBEKF_X_d,  SensorsPoint(3,:))).^2 + ...
                        0.5 * tmpBIWVBEKF_BerZt3_d * diag((H(3,:) * BIWVBEKF_Px_d * H(3,:)')); 
                    BIWVBEKF_BetaEt3_d = BIWVBEKF_BetaE0_d + tmpBIWVBEKF_BerZt3_d;
                    BIWVBEKF_BetaFt3_d = BIWVBEKF_BetaF0_d + 1 - tmpBIWVBEKF_BerZt3_d; % VB-M step done 
                else
                    BIWVBEKF_IGb3_d = tmpBIWVBEKF_IGb3_d; 
                end
                if tmpBIWVBEKF_BerZt4_d > BIWVBEKF_BerZt_TH 
                    BIWVBEKF_Bt = (Ypse(4,k) - Ht(BIWVBEKF_X_d, SensorsPoint(4,:))) * (Ypse(4,k) - Ht(BIWVBEKF_X_d, SensorsPoint(4,:)))' +  H(4,:) * BIWVBEKF_Px_d * H(4,:)';
                    tmpBIWVBEKF_BerZt4_d = exp(psi(BIWVBEKF_BetaEt4_d) - psi(BIWVBEKF_BetaEt4_d + BIWVBEKF_BetaFt4_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa4_d / BIWVBEKF_IGb4_d)) / ...
                             (exp(psi(BIWVBEKF_BetaEt4_d) - psi(BIWVBEKF_BetaEt4_d + BIWVBEKF_BetaFt4_d ) - 0.5 * trace(BIWVBEKF_Bt * BIWVBEKF_IGa4_d / BIWVBEKF_IGb4_d)) + ...
                                exp(psi(BIWVBEKF_BetaFt4_d ) - psi(BIWVBEKF_BetaEt4_d + BIWVBEKF_BetaFt4_d )));
                    BIWVBEKF_IGb4_d = tmpBIWVBEKF_IGb4_d +  0.5 * tmpBIWVBEKF_BerZt4_d * (Ypse(4,k) - Ht(BIWVBEKF_X_d,  SensorsPoint(4,:))).^2 + ...
                        0.5 * tmpBIWVBEKF_BerZt4_d * diag((H(4,:) * BIWVBEKF_Px_d * H(4,:)')); 
                    BIWVBEKF_BetaEt4_d = BIWVBEKF_BetaE0_d + tmpBIWVBEKF_BerZt4_d;
                    BIWVBEKF_BetaFt4_d = BIWVBEKF_BetaF0_d + 1 - tmpBIWVBEKF_BerZt4_d; % VB-M step done 
                else 
                    BIWVBEKF_IGb4_d = tmpBIWVBEKF_IGb4_d; 
                end
                if(norm(BIWVBEKF_X_d - BIWVBEKF_X_d_pre) < BIWVBEKF_E_d)
                    break;
                end
            else
                BIWVBEKF_X_d = tmpBIWVBEKF_X_d; 
                BIWVBEKF_Px_d = tmpBIWVBEKF_Px_d; 
                BIWVBEKF_IGb1_d = tmpBIWVBEKF_IGb1_d; 
                BIWVBEKF_IGb2_d = tmpBIWVBEKF_IGb2_d; 
                BIWVBEKF_IGb3_d = tmpBIWVBEKF_IGb3_d; 
                BIWVBEKF_IGb4_d = tmpBIWVBEKF_IGb4_d; 
                break;
            end
        end
        if tmpBIWVBEKF_BerZt1_d <= BIWVBEKF_BerZt_TH 
            BIWVBEKF_IGa1_d =  BIWVBEKF_IGa1_d - 0.5; 
        end
        if tmpBIWVBEKF_BerZt2_d <= BIWVBEKF_BerZt_TH 
            BIWVBEKF_IGa2_d =  BIWVBEKF_IGa2_d - 0.5; 
        end
        if tmpBIWVBEKF_BerZt3_d <= BIWVBEKF_BerZt_TH 
            BIWVBEKF_IGa3_d =  BIWVBEKF_IGa3_d - 0.5; 
        end
        if tmpBIWVBEKF_BerZt4_d <= BIWVBEKF_BerZt_TH 
            BIWVBEKF_IGa4_d =  BIWVBEKF_IGa4_d - 0.5; 
        end
        BIWVBEKF_R_d = diag([BIWVBEKF_IGb1_d / BIWVBEKF_IGa1_d, BIWVBEKF_IGb2_d / BIWVBEKF_IGa2_d,...
            BIWVBEKF_IGb3_d / BIWVBEKF_IGa3_d, BIWVBEKF_IGb4_d / BIWVBEKF_IGa4_d]); 
        resBIWVBEKF_it_d(:,k) = i;
        %res
        resBIWVBEKF_X_d(:,k) = BIWVBEKF_X_d;
        resBIWVBEKF_R_d(:,k) = diag(BIWVBEKF_R_d);
        resBIWVBZt(:,k) = [tmpBIWVBEKF_BerZt1_d; tmpBIWVBEKF_BerZt2_d;...
             tmpBIWVBEKF_BerZt3_d; tmpBIWVBEKF_BerZt4_d];
        resBIWVBEKF_RMSE_d(:,k) = sqrt(trace(BIWVBEKF_Px_d));
        k=k+1;
    end   

%% BIWVBEKF (bern inv Gamma) SEQ - MINE
    resBIWVBEKFSEQ_X_d = zeros(5, fix(stopT / deltaT));
    resBIWVBEKFSEQ_R_d = zeros(4, fix(stopT / deltaT));
    resBIWVBEKFSEQ_RMSE_d = zeros(1, fix(stopT / deltaT));
    resBIWVBKFSEQ_1st_it_d = zeros(1, fix(stopT / deltaT));
    resBIWVBKFSEQ_2nd_it_d = zeros(1, fix(stopT / deltaT));
    resBIWVBKFSEQ_3rd_it_d = zeros(1, fix(stopT / deltaT));
    resBIWVBKFSEQ_4th_it_d = zeros(1, fix(stopT / deltaT));
    resBIWVBSEQZt = zeros(4, fix(stopT / deltaT));
    BIWVBEKFSEQ_X_d = filter_X0;
    BIWVBEKFSEQ_Px_d = filter_P0;  
    BIWVBEKFSEQ_BerZt_d = 1; % Bernoulli param. 
    BIWVBEKFSEQ_BerZt_TH = 0.1;
    BIWVBEKFSEQ_BetaE0_d = 0.9; BIWVBEKFSEQ_BetaF0_d = 0.1; % Beta params.
    BIWVBEKFSEQ_IGa1_d = 1; BIWVBEKFSEQ_IGb1_d = R(1,1);%BIWVBEKFSEQ_IGb1_d = 50^2; % Inv-Gamma param.
    BIWVBEKFSEQ_IGa2_d = 1; BIWVBEKFSEQ_IGb2_d = R(2,1);%BIWVBEKFSEQ_IGb2_d = 50^2; % Inv-Gamma param.
    BIWVBEKFSEQ_IGa3_d = 1; BIWVBEKFSEQ_IGb3_d = R(3,1);%BIWVBEKFSEQ_IGb3_d = 50^2; % Inv-Gamma param.
    BIWVBEKFSEQ_IGa4_d = 1; BIWVBEKFSEQ_IGb4_d = R(4,1);%BIWVBEKFSEQ_IGb4_d = 50^2; % Inv-Gamma param.
%     BIWVBEKFSEQ_it_max_d = 20; % iteration times
    BIWVBEKFSEQ_it_max_d = VB_it_Max;
    BIWVBEKFSEQ_E_d = VB_D_Err_TH; % error threshold
    BIWVBEKFSEQ_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        % time update
        F = JabF(BIWVBEKFSEQ_X_d, deltaT);
        BIWVBEKFSEQ_X_d = Ft(BIWVBEKFSEQ_X_d, deltaT);
        BIWVBEKFSEQ_Px_d = F*BIWVBEKFSEQ_Px_d*F'+ Q;
        BIWVBEKFSEQ_IGa1_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGa1_d; BIWVBEKFSEQ_IGa2_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGa2_d; % heuristic dynamic
        BIWVBEKFSEQ_IGb1_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGb1_d; BIWVBEKFSEQ_IGb2_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGb2_d; % heuristic dynamic
        BIWVBEKFSEQ_IGa3_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGa3_d; BIWVBEKFSEQ_IGa4_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGa4_d; % heuristic dynamic
        BIWVBEKFSEQ_IGb3_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGb3_d; BIWVBEKFSEQ_IGb4_d = BIWVBEKFSEQ_pho_d * BIWVBEKFSEQ_IGb4_d; % heuristic dynamic
        % measurement update
        % Sensors 1
        H = JabH(BIWVBEKFSEQ_X_d, SensorsPoint); 
        H1 = H(1,:); H2 = H(2,:); H3 = H(3,:); H4 = H(4,:); 
%         H1 = JabH(BIWVBEKFSEQ_X_d, SensorsPoint(1,:)); 
%         BIWVBEKFSEQ_Hk1_d = H1; 
        BIWVBEKFSEQ_IGa1_d = 0.5 + BIWVBEKFSEQ_IGa1_d; % VB step1 
        tmpBIWVBEKFSEQ_BerZ1t_d = BIWVBEKFSEQ_BerZt_d; 
        BIWVBEKFSEQ_BetaE1t_d = BIWVBEKFSEQ_BetaE0_d;  
        BIWVBEKFSEQ_BetaF1t_d = BIWVBEKFSEQ_BetaF0_d; 
        tmpBIWVBEKFSEQ_X_d = BIWVBEKFSEQ_X_d; 
        tmpBIWVBEKFSEQ_Px_d = BIWVBEKFSEQ_Px_d; 
        tmpBIWVBEKFSEQ_IGb1_d = BIWVBEKFSEQ_IGb1_d; 
        % Sensors 1
        for i = 1 : BIWVBEKFSEQ_it_max_d % VB iterate dynamic times
            BIWVBEKFSEQ_Hk1_d = H1;
            if tmpBIWVBEKFSEQ_BerZ1t_d > BIWVBEKFSEQ_BerZt_TH 
                BIWVBEKFSEQ_X_d_pre = BIWVBEKFSEQ_X_d; 
                BIWVBEKFSEQ_R1_d = BIWVBEKFSEQ_IGb1_d / BIWVBEKFSEQ_IGa1_d; % VB-E step begin
                BIWVBEKFSEQ_Sk_d = BIWVBEKFSEQ_Hk1_d * tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk1_d'+ BIWVBEKFSEQ_R1_d / tmpBIWVBEKFSEQ_BerZ1t_d; 
                BIWVBEKFSEQ_Kk_d = tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk1_d' / BIWVBEKFSEQ_Sk_d; 
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d + BIWVBEKFSEQ_Kk_d * (Ypse(1,k) - Ht(tmpBIWVBEKFSEQ_X_d, SensorsPoint(1,:))); 
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d - BIWVBEKFSEQ_Kk_d * BIWVBEKFSEQ_Sk_d * BIWVBEKFSEQ_Kk_d'; % VB-E step done 
                BIWVBEKFSEQ_Bt = (Ypse(1,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(1,:))) * (Ypse(1,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(1,:)))' + BIWVBEKFSEQ_Hk1_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk1_d'; 
                tmpBIWVBEKFSEQ_BerZ1t_d = exp(psi(BIWVBEKFSEQ_BetaE1t_d) - psi(BIWVBEKFSEQ_BetaE1t_d + BIWVBEKFSEQ_BetaF1t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R1_d)) / ... 
                    (exp(psi(BIWVBEKFSEQ_BetaE1t_d) - psi(BIWVBEKFSEQ_BetaE1t_d + BIWVBEKFSEQ_BetaF1t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R1_d)) + ... 
                    exp(psi(BIWVBEKFSEQ_BetaF1t_d ) - psi(BIWVBEKFSEQ_BetaE1t_d + BIWVBEKFSEQ_BetaF1t_d )));  
                BIWVBEKFSEQ_IGb1_d = tmpBIWVBEKFSEQ_IGb1_d + ... % VB-M step begin 
                    0.5 * tmpBIWVBEKFSEQ_BerZ1t_d * (Ypse(1,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(1,:))).^2 + ... 
                    0.5 * tmpBIWVBEKFSEQ_BerZ1t_d * diag((BIWVBEKFSEQ_Hk1_d*BIWVBEKFSEQ_Px_d*BIWVBEKFSEQ_Hk1_d')); 
                BIWVBEKFSEQ_BetaE1t_d = BIWVBEKFSEQ_BetaE0_d + tmpBIWVBEKFSEQ_BerZ1t_d; 
                BIWVBEKFSEQ_BetaF1t_d = BIWVBEKFSEQ_BetaF0_d + 1 - tmpBIWVBEKFSEQ_BerZ1t_d; % VB-M step done 
                if(norm(BIWVBEKFSEQ_X_d - BIWVBEKFSEQ_X_d_pre) < BIWVBEKFSEQ_E_d) 
                    break;
                end
            else
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d;
                BIWVBEKFSEQ_IGb1_d = tmpBIWVBEKFSEQ_IGb1_d; 
                BIWVBEKFSEQ_IGa1_d = BIWVBEKFSEQ_IGa1_d - 0.5;
                BIWVBEKFSEQ_R1_d = BIWVBEKFSEQ_IGb1_d / BIWVBEKFSEQ_IGa1_d; 
                break;
            end
        end
        resBIWVBKFSEQ_1st_it_d(:,k) = i;
        % Sensors 2
        BIWVBEKFSEQ_IGa2_d = 0.5 + BIWVBEKFSEQ_IGa2_d; % VB step1
        tmpBIWVBEKFSEQ_BerZ2t_d = BIWVBEKFSEQ_BerZt_d; 
        BIWVBEKFSEQ_BetaE2t_d = BIWVBEKFSEQ_BetaE0_d; 
        BIWVBEKFSEQ_BetaF2t_d = BIWVBEKFSEQ_BetaF0_d;
        tmpBIWVBEKFSEQ_X_d = BIWVBEKFSEQ_X_d; 
        tmpBIWVBEKFSEQ_Px_d = BIWVBEKFSEQ_Px_d; 
        tmpBIWVBEKFSEQ_IGb2_d = BIWVBEKFSEQ_IGb2_d; 
        for i = 1 : BIWVBEKFSEQ_it_max_d % VB iterate dynamic times
            BIWVBEKFSEQ_Hk2_d = H2;
            if tmpBIWVBEKFSEQ_BerZ2t_d > BIWVBEKFSEQ_BerZt_TH
                BIWVBEKFSEQ_X_d_pre = BIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_R2_d = BIWVBEKFSEQ_IGb2_d / BIWVBEKFSEQ_IGa2_d; % VB-E step begin
                BIWVBEKFSEQ_Sk_d = BIWVBEKFSEQ_Hk2_d * tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk2_d'+ BIWVBEKFSEQ_R2_d / tmpBIWVBEKFSEQ_BerZ2t_d;
                BIWVBEKFSEQ_Kk_d = tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk2_d' / BIWVBEKFSEQ_Sk_d;
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d + BIWVBEKFSEQ_Kk_d * (Ypse(2,k) - Ht(tmpBIWVBEKFSEQ_X_d, SensorsPoint(2,:)));
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d - BIWVBEKFSEQ_Kk_d * BIWVBEKFSEQ_Sk_d * BIWVBEKFSEQ_Kk_d'; % VB-E step done
                BIWVBEKFSEQ_Bt = (Ypse(2,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(2,:))) * (Ypse(2,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(2,:)))' + BIWVBEKFSEQ_Hk2_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk2_d';
                tmpBIWVBEKFSEQ_BerZ2t_d = exp(psi(BIWVBEKFSEQ_BetaE2t_d) - psi(BIWVBEKFSEQ_BetaE2t_d + BIWVBEKFSEQ_BetaF2t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R2_d)) / ...
                    (exp(psi(BIWVBEKFSEQ_BetaE2t_d) - psi(BIWVBEKFSEQ_BetaE2t_d + BIWVBEKFSEQ_BetaF2t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R2_d)) + ...
                    exp(psi(BIWVBEKFSEQ_BetaF2t_d ) - psi(BIWVBEKFSEQ_BetaE2t_d + BIWVBEKFSEQ_BetaF2t_d )));
                BIWVBEKFSEQ_IGb2_d = tmpBIWVBEKFSEQ_IGb2_d + ... % VB-M step begin 
                    0.5 * tmpBIWVBEKFSEQ_BerZ2t_d * (Ypse(2,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(2,:))).^2 + ... 
                    0.5 * tmpBIWVBEKFSEQ_BerZ2t_d * diag((BIWVBEKFSEQ_Hk2_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk2_d')); 
                BIWVBEKFSEQ_BetaE2t_d = BIWVBEKFSEQ_BetaE0_d + tmpBIWVBEKFSEQ_BerZ2t_d; 
                BIWVBEKFSEQ_BetaF2t_d = BIWVBEKFSEQ_BetaF0_d + 1 - tmpBIWVBEKFSEQ_BerZ2t_d; % VB-M step done
                if(norm(BIWVBEKFSEQ_X_d - BIWVBEKFSEQ_X_d_pre) < BIWVBEKFSEQ_E_d)
                    break;
                end
            else
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d; 
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d; 
                BIWVBEKFSEQ_IGb2_d = tmpBIWVBEKFSEQ_IGb2_d; 
                BIWVBEKFSEQ_IGa2_d = BIWVBEKFSEQ_IGa2_d - 0.5; 
                BIWVBEKFSEQ_R2_d = BIWVBEKFSEQ_IGb2_d / BIWVBEKFSEQ_IGa2_d; 
                break;
            end
        end
        resBIWVBKFSEQ_2nd_it_d(:,k) = i;
        % Sensors 3
        BIWVBEKFSEQ_IGa3_d = 0.5 + BIWVBEKFSEQ_IGa3_d; % VB step1
        tmpBIWVBEKFSEQ_BerZ3t_d = BIWVBEKFSEQ_BerZt_d;
        BIWVBEKFSEQ_BetaE3t_d = BIWVBEKFSEQ_BetaE0_d; 
        BIWVBEKFSEQ_BetaF3t_d = BIWVBEKFSEQ_BetaF0_d; 
        tmpBIWVBEKFSEQ_X_d = BIWVBEKFSEQ_X_d; 
        tmpBIWVBEKFSEQ_Px_d = BIWVBEKFSEQ_Px_d; 
        tmpBIWVBEKFSEQ_IGb3_d = BIWVBEKFSEQ_IGb3_d; 
        for i = 1 : BIWVBEKFSEQ_it_max_d % VB iterate dynamic times
            BIWVBEKFSEQ_Hk3_d = H3;
            if tmpBIWVBEKFSEQ_BerZ3t_d > BIWVBEKFSEQ_BerZt_TH
                BIWVBEKFSEQ_X_d_pre = BIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_R3_d =  BIWVBEKFSEQ_IGb3_d / BIWVBEKFSEQ_IGa3_d; % VB-E step begin
                BIWVBEKFSEQ_Sk_d = BIWVBEKFSEQ_Hk3_d * tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk3_d'+ BIWVBEKFSEQ_R3_d / tmpBIWVBEKFSEQ_BerZ3t_d;
                BIWVBEKFSEQ_Kk_d = tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk3_d' / BIWVBEKFSEQ_Sk_d;
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d + BIWVBEKFSEQ_Kk_d * (Ypse(3,k) - Ht(tmpBIWVBEKFSEQ_X_d, SensorsPoint(3,:)));
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d - BIWVBEKFSEQ_Kk_d * BIWVBEKFSEQ_Sk_d * BIWVBEKFSEQ_Kk_d'; % VB-E step done
                BIWVBEKFSEQ_Bt = (Ypse(3,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(3,:))) * (Ypse(3,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(3,:)))' + BIWVBEKFSEQ_Hk3_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk3_d';
                tmpBIWVBEKFSEQ_BerZ3t_d = exp(psi(BIWVBEKFSEQ_BetaE3t_d) - psi(BIWVBEKFSEQ_BetaE3t_d + BIWVBEKFSEQ_BetaF3t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R3_d)) / ...
                    (exp(psi(BIWVBEKFSEQ_BetaE3t_d) - psi(BIWVBEKFSEQ_BetaE3t_d + BIWVBEKFSEQ_BetaF3t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R3_d)) + ...
                    exp(psi(BIWVBEKFSEQ_BetaF3t_d ) - psi(BIWVBEKFSEQ_BetaE3t_d + BIWVBEKFSEQ_BetaF3t_d )));
                BIWVBEKFSEQ_IGb3_d =  tmpBIWVBEKFSEQ_IGb3_d + ... % VB-M step begin
                    0.5 * tmpBIWVBEKFSEQ_BerZ3t_d * (Ypse(3,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(3,:))).^2 + ...
                    0.5 * tmpBIWVBEKFSEQ_BerZ3t_d * diag((BIWVBEKFSEQ_Hk3_d*BIWVBEKFSEQ_Px_d*BIWVBEKFSEQ_Hk3_d'));
                BIWVBEKFSEQ_BetaE3t_d = BIWVBEKFSEQ_BetaE0_d + tmpBIWVBEKFSEQ_BerZ3t_d;
                BIWVBEKFSEQ_BetaF3t_d = BIWVBEKFSEQ_BetaF0_d + 1 - tmpBIWVBEKFSEQ_BerZ3t_d; % VB-M step done
                if(norm(BIWVBEKFSEQ_X_d - BIWVBEKFSEQ_X_d_pre) < BIWVBEKFSEQ_E_d)
                    break;
                end
            else
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d;
                BIWVBEKFSEQ_IGb3_d = tmpBIWVBEKFSEQ_IGb3_d; 
                BIWVBEKFSEQ_IGa3_d = BIWVBEKFSEQ_IGa3_d - 0.5;
                BIWVBEKFSEQ_R3_d = BIWVBEKFSEQ_IGb3_d/BIWVBEKFSEQ_IGa3_d;
                break;
            end
        end
        resBIWVBKFSEQ_3rd_it_d(:,k) = i;
        % Sensors 4
        BIWVBEKFSEQ_IGa4_d = 0.5 + BIWVBEKFSEQ_IGa4_d; % VB step1
        tmpBIWVBEKFSEQ_BerZ4t_d = BIWVBEKFSEQ_BerZt_d;
        BIWVBEKFSEQ_BetaE4t_d = BIWVBEKFSEQ_BetaE0_d; 
        BIWVBEKFSEQ_BetaF4t_d = BIWVBEKFSEQ_BetaF0_d;
        tmpBIWVBEKFSEQ_X_d = BIWVBEKFSEQ_X_d; 
        tmpBIWVBEKFSEQ_Px_d = BIWVBEKFSEQ_Px_d; 
        tmpBIWVBEKFSEQ_IGb4_d = BIWVBEKFSEQ_IGb4_d;
        for i = 1 : BIWVBEKFSEQ_it_max_d % VB iterate dynamic times
%             H4 = JabH(BIWVBEKFSEQ_X_d, SensorsPoint(4,:));
            BIWVBEKFSEQ_Hk4_d = H4;
            if tmpBIWVBEKFSEQ_BerZ4t_d > BIWVBEKFSEQ_BerZt_TH
                BIWVBEKFSEQ_X_d_pre = BIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_R4_d =  BIWVBEKFSEQ_IGb4_d / BIWVBEKFSEQ_IGa4_d; % VB-E step begin
                BIWVBEKFSEQ_Sk_d = BIWVBEKFSEQ_Hk4_d * tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk4_d'+ BIWVBEKFSEQ_R4_d / tmpBIWVBEKFSEQ_BerZ4t_d;
                BIWVBEKFSEQ_Kk_d = tmpBIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk4_d' / BIWVBEKFSEQ_Sk_d;
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d + BIWVBEKFSEQ_Kk_d * (Ypse(4,k) - Ht(tmpBIWVBEKFSEQ_X_d, SensorsPoint(4,:)));
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d - BIWVBEKFSEQ_Kk_d * BIWVBEKFSEQ_Sk_d * BIWVBEKFSEQ_Kk_d'; % VB-E step done
                BIWVBEKFSEQ_Bt = (Ypse(4,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(4,:))) * (Ypse(4,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(4,:)))' + BIWVBEKFSEQ_Hk4_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk4_d';
                tmpBIWVBEKFSEQ_BerZ4t_d = exp(psi(BIWVBEKFSEQ_BetaE4t_d) - psi(BIWVBEKFSEQ_BetaE4t_d + BIWVBEKFSEQ_BetaF4t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R4_d)) / ...
                    (exp(psi(BIWVBEKFSEQ_BetaE4t_d) - psi(BIWVBEKFSEQ_BetaE4t_d + BIWVBEKFSEQ_BetaF4t_d ) - 0.5 * trace(BIWVBEKFSEQ_Bt / BIWVBEKFSEQ_R4_d)) + ...
                    exp(psi(BIWVBEKFSEQ_BetaF4t_d ) - psi(BIWVBEKFSEQ_BetaE4t_d + BIWVBEKFSEQ_BetaF4t_d )));
                BIWVBEKFSEQ_IGb4_d = tmpBIWVBEKFSEQ_IGb4_d + ... % VB-M step begin
                    0.5 * tmpBIWVBEKFSEQ_BerZ4t_d * (Ypse(4,k) - Ht(BIWVBEKFSEQ_X_d, SensorsPoint(4,:))).^2 + ...
                    0.5 * tmpBIWVBEKFSEQ_BerZ4t_d * diag((BIWVBEKFSEQ_Hk4_d * BIWVBEKFSEQ_Px_d * BIWVBEKFSEQ_Hk4_d'));
                BIWVBEKFSEQ_BetaE4t_d = BIWVBEKFSEQ_BetaE0_d + tmpBIWVBEKFSEQ_BerZ4t_d;
                BIWVBEKFSEQ_BetaF4t_d = BIWVBEKFSEQ_BetaF0_d + 1 - tmpBIWVBEKFSEQ_BerZ4t_d; % VB-M step done
                if(norm(BIWVBEKFSEQ_X_d - BIWVBEKFSEQ_X_d_pre) < BIWVBEKFSEQ_E_d)
                    break;
                end
            else
                BIWVBEKFSEQ_X_d = tmpBIWVBEKFSEQ_X_d;
                BIWVBEKFSEQ_Px_d = tmpBIWVBEKFSEQ_Px_d;
                BIWVBEKFSEQ_IGb4_d = tmpBIWVBEKFSEQ_IGb4_d; 
                BIWVBEKFSEQ_IGa4_d = BIWVBEKFSEQ_IGa4_d - 0.5;
                BIWVBEKFSEQ_R4_d = BIWVBEKFSEQ_IGb4_d / BIWVBEKFSEQ_IGa4_d;
                break;
            end
        end
        resBIWVBKFSEQ_4th_it_d(:,k) = i;
        %res
        resBIWVBKFSEQ_it_d = resBIWVBKFSEQ_1st_it_d + resBIWVBKFSEQ_2nd_it_d + ...
            resBIWVBKFSEQ_3rd_it_d + resBIWVBKFSEQ_4th_it_d;
        resBIWVBEKFSEQ_X_d(:,k) = BIWVBEKFSEQ_X_d;
        resBIWVBEKFSEQ_R_d(:,k) = [BIWVBEKFSEQ_R1_d; BIWVBEKFSEQ_R2_d;...
            BIWVBEKFSEQ_R3_d; BIWVBEKFSEQ_R4_d];
        resBIWVBSEQZt(:,k) = [tmpBIWVBEKFSEQ_BerZ1t_d; tmpBIWVBEKFSEQ_BerZ2t_d;...
            tmpBIWVBEKFSEQ_BerZ3t_d; tmpBIWVBEKFSEQ_BerZ4t_d];
        resBIWVBEKFSEQ_RMSE_d(:,k) = sqrt(trace(BIWVBEKFSEQ_Px_d));
        k=k+1;
    end 

%% CRLB
    % cubature integration
    if mc == MC_times
        resCubCRLB_trLB = zeros(1, fix(stopT / deltaT));
        CubCRLB_J = inv(filter_P0);
        CubCRLB_X = Xpse;
        CubCRLB_N = length(CubCRLB_X(:, 1));
        k = 1;
        for t = 0 : deltaT : stopT - deltaT 
            Sk = sqrt(CubCRLB_N) * chol(inv(CubCRLB_J))';
            Xn = repmat(CubCRLB_X(:, k), 1, CubCRLB_N); 
            SigmaPoint = [Xn + Sk, Xn - Sk]; 
            CubCRLB_D11 = zeros(CubCRLB_N, CubCRLB_N); 
            CubCRLB_D12 = zeros(CubCRLB_N, CubCRLB_N);
            for i = 1 : 2 * CubCRLB_N
                F = JabF(SigmaPoint(:, i), deltaT); 
                CubCRLB_D11 = CubCRLB_D11 + (1 / (2 * CubCRLB_N)) * (F' / Q) * F; 
                CubCRLB_D12 = CubCRLB_D12 - (1 / (2 * CubCRLB_N)) * F' / Q; 
            end
            H = JabH(CubCRLB_X(:, k + 1), SensorsPoint);
            CubCRLB_D22 = inv(Q) + (H' / (diag(R(:, k)))) * H; 
            CubCRLB_J = CubCRLB_D22 - (CubCRLB_D12' / (CubCRLB_J + CubCRLB_D11)) * CubCRLB_D12;
            %res
            resCubCRLB_trLB(:,k) = sqrt(trace(inv(CubCRLB_J))); 
            k = k + 1;
        end
    end

%% RMSE Evaluate 
    % MC EKF result
    RMSE_CHIEKF_X(mc, 1) = rmse(resChiEKF_X(1, :), Xpse(1, 2 : end));
    RMSE_CHIEKF_X(mc, 2) = rmse(resChiEKF_X(2, :), Xpse(2, 2 : end));
    RMSE_CHIEKF_X(mc, 3) = rmse(resChiEKF_X(3, :), Xpse(3, 2 : end));
    RMSE_CHIEKF_X(mc, 4) = rmse(resChiEKF_X(4, :), Xpse(4, 2 : end));
    RMSE_CHIEKF_X(mc, 5) = rmse(resChiEKF_X(5, :), Xpse(5, 2 : end));
    % MC VBEKF result
    RMSE_VBEKF_X(mc, 1) = rmse(resVBEKF_X_d(1, :), Xpse(1, 2 : end));
    RMSE_VBEKF_X(mc, 2) = rmse(resVBEKF_X_d(2, :), Xpse(2, 2 : end));
    RMSE_VBEKF_X(mc, 3) = rmse(resVBEKF_X_d(3, :), Xpse(3, 2 : end));
    RMSE_VBEKF_X(mc, 4) = rmse(resVBEKF_X_d(4, :), Xpse(4, 2 : end));
    RMSE_VBEKF_X(mc, 5) = rmse(resVBEKF_X_d(5, :), Xpse(5, 2 : end));
    RMSE_VBEKF_R(mc, 1) = rmse(resVBEKF_R_d(1, :), R(1, :));
    RMSE_VBEKF_R(mc, 2) = rmse(resVBEKF_R_d(2, :), R(2, :));
    RMSE_VBEKF_R(mc, 3) = rmse(resVBEKF_R_d(3, :), R(3, :));
    RMSE_VBEKF_R(mc, 4) = rmse(resVBEKF_R_d(4, :), R(4, :));
    % MC BIWVBEKF result
    RMSE_BIWVBEKF_X(mc, 1) = rmse(resBIWVBEKF_X_d(1, :), Xpse(1, 2 : end));
    RMSE_BIWVBEKF_X(mc, 2) = rmse(resBIWVBEKF_X_d(2, :), Xpse(2, 2 : end));
    RMSE_BIWVBEKF_X(mc, 3) = rmse(resBIWVBEKF_X_d(3, :), Xpse(3, 2 : end));
    RMSE_BIWVBEKF_X(mc, 4) = rmse(resBIWVBEKF_X_d(4, :), Xpse(4, 2 : end));
    RMSE_BIWVBEKF_X(mc, 5) = rmse(resBIWVBEKF_X_d(5, :), Xpse(5, 2 : end));
    RMSE_BIWVBEKF_R(mc, 1) = rmse(resBIWVBEKF_R_d(1, :), R(1, :));
    RMSE_BIWVBEKF_R(mc, 2) = rmse(resBIWVBEKF_R_d(2, :), R(2, :));
    RMSE_BIWVBEKF_R(mc, 3) = rmse(resBIWVBEKF_R_d(3, :), R(3, :));
    RMSE_BIWVBEKF_R(mc, 4) = rmse(resBIWVBEKF_R_d(4, :), R(4, :));
    % MC BIWVBEKF result
    RMSE_BIWVBEKFSEQ_X(mc, 1) = rmse(resBIWVBEKFSEQ_X_d(1, :), Xpse(1, 2 : end));
    RMSE_BIWVBEKFSEQ_X(mc, 2) = rmse(resBIWVBEKFSEQ_X_d(2, :), Xpse(2, 2 : end));
    RMSE_BIWVBEKFSEQ_X(mc, 3) = rmse(resBIWVBEKFSEQ_X_d(3, :), Xpse(3, 2 : end));
    RMSE_BIWVBEKFSEQ_X(mc, 4) = rmse(resBIWVBEKFSEQ_X_d(4, :), Xpse(4, 2 : end));
    RMSE_BIWVBEKFSEQ_X(mc, 5) = rmse(resBIWVBEKFSEQ_X_d(5, :), Xpse(5, 2 : end));
    RMSE_BIWVBEKFSEQ_R(mc, 1) = rmse(resBIWVBEKFSEQ_R_d(1, :), R(1, :));
    RMSE_BIWVBEKFSEQ_R(mc, 2) = rmse(resBIWVBEKFSEQ_R_d(2, :), R(2, :));
    RMSE_BIWVBEKFSEQ_R(mc, 3) = rmse(resBIWVBEKFSEQ_R_d(3, :), R(3, :));
    RMSE_BIWVBEKFSEQ_R(mc, 4) = rmse(resBIWVBEKFSEQ_R_d(4, :), R(4, :));

end

%% plot
f1 = figure('Name','f1_true model');
subplot(5,4,[1,2,5,6]);
% plot(Xpse(1,:), Xpse(3,:),'-'); hold on;
plot(Xpse(1,:), Xpse(3,:),'-'); hold on;
plot(resChiEKF_X(1,:), resChiEKF_X(3,:),'-'); hold on;
plot(resVBEKF_X_d(1,:), resVBEKF_X_d(3,:),'-'); hold on;
plot(resBIWVBEKF_X_d(1,:), resBIWVBEKF_X_d(3,:),'-'); hold on;
plot(resBIWVBEKFSEQ_X_d(1,:), resBIWVBEKFSEQ_X_d(3,:),'-'); hold on;
plot(SensorsPoint(:,1), SensorsPoint(:,2),'pentagram');
legend('True', 'EKF', 'VBEKF', 'BIGVBEKF', 'BIGVBEKFSEQ');
set(ylabel(['$v$($m$)'],'Interpreter','latex'));
set(xlabel(['$u$($m$)'],'Interpreter','latex'));
subplot(5,4,[3,4,7,8]);
plot(Time,Ypse);
set(ylabel(['measurement value ($m$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(5,4,[9,10,11,12]);
plot(Time, Xpse(2,2:end),'-'); hold on;
set(ylabel(['$\dot{u}$($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(5,4,[13,14,15,16]);
plot(Time, Xpse(4,2:end),'-'); hold on;
set(ylabel(['$\dot{v}$($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(5,4,[17,18,19,20]);
plot(Time, Xpse(5,2:end),'-'); hold on;
set(ylabel(['$\omega$($rad/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
sgtitle("True Trajectory",'Fontsize',10,'Interpreter','Latex');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f2 = figure('Name','f2_estimated states');
plot(Xpse(1,:), Xpse(3,:),'-'); hold on;
plot(resChiEKF_X(1,:), resChiEKF_X(3,:),'-'); hold on;
plot(resVBEKF_X_d(1,:), resVBEKF_X_d(3,:),'-'); hold on;
plot(resBIWVBEKF_X_d(1,:), resBIWVBEKF_X_d(3,:),'-'); hold on;
plot(resBIWVBEKFSEQ_X_d(1,:), resBIWVBEKFSEQ_X_d(3,:),'-'); hold on;
set(ylabel(['$v$($m$)'],'Interpreter','latex'));
set(xlabel(['$u$($m$)'],'Interpreter','latex'));
legend('True', 'EKF', 'VBEKF', 'BIGVBEKF', 'BIGVBEKFSEQ');
title("Filter Result",'Fontsize',10,'Interpreter','Latex');
pos = get(f2,'Position');
set(f2,'Units','Inches');
set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f3 = figure('Name','f3_estimated R');
subplot(411); plot(Time,R(1,:)); hold on; plot(Time,resVBEKF_R_d(1,:)); plot(Time,resBIWVBEKF_R_d(1,:)); plot(Time,resBIWVBEKFSEQ_R_d(1,:)); 
set(ylabel(['$\hat{R}_1$'],'Interpreter','latex'));
subplot(412); plot(Time,R(2,:)); hold on; plot(Time,resVBEKF_R_d(2,:)); plot(Time,resBIWVBEKF_R_d(2,:)); plot(Time,resBIWVBEKFSEQ_R_d(2,:)); 
set(ylabel(['$\hat{R}_2$'],'Interpreter','latex'));
subplot(413); plot(Time,R(3,:)); hold on; plot(Time,resVBEKF_R_d(3,:)); plot(Time,resBIWVBEKF_R_d(3,:)); plot(Time,resBIWVBEKFSEQ_R_d(3,:)); 
set(ylabel(['$\hat{R}_3$'],'Interpreter','latex'));
subplot(414); plot(Time,R(4,:)); hold on; plot(Time,resVBEKF_R_d(4,:)); plot(Time,resBIWVBEKF_R_d(4,:)); plot(Time,resBIWVBEKFSEQ_R_d(4,:)); 
set(ylabel(['$\hat{R}_4$'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
legend('True', 'VBEKF', 'BIGVBEKF', 'BIGVBEKFSEQ');
sgtitle("Estimated Measurement Noise",'Fontsize',10,'Interpreter','Latex');
pos = get(f3,'Position');
set(f3,'Units','Inches');
set(f3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

% plot CRLB and RMSE
f4 = figure('Name','f4_CRLB and RMSE'); 
plot(Time, [resEKF_RMSE;resVBEKF_RMSE_d;resBIWVBEKF_RMSE_d;resBIWVBEKFSEQ_RMSE_d;resCubCRLB_trLB]'); 
legend('ChiEKF', 'VBEKF', 'BIWVBEKF', 'BIWVBEKFSEQ', 'CRLB'); 
set(ylabel(['RMSE error'],'Interpreter','latex')); 
set(xlabel(['Time($s$)'],'Interpreter','latex')); 
title("CRLB and RMSE",'Fontsize',10,'Interpreter','Latex'); 
pos = get(f4,'Position');
set(f4,'Units','Inches');
set(f4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f5 = figure('Name','f5_iteration times'); 
plot(Time, [resBIWVBEKF_it_d;resBIWVBKFSEQ_it_d]); 
legend('BIGVBEKF', 'BIGVBEKFSEQ'); 
set(ylabel(['Iteration Times'],'Interpreter','latex')); 
set(xlabel(['Time($s$)'],'Interpreter','latex')); 
title("Iteration Times",'Fontsize',10,'Interpreter','Latex'); 
pos = get(f5,'Position');
set(f5,'Units','Inches');
set(f5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f6 = figure('Name','f6_Bern Zt index');
subplot(3, 1, 1);
pcolor(resBIWVBZt);
set(ylabel('BIGVBEKF','Interpreter','latex'));
subplot(3, 1, 2);
pcolor(resBIWVBSEQZt);
set(ylabel('BIGVBEKFSEQ','Interpreter','latex'));
subplot(3, 1, 3);
pcolor(resSEQEKFCHIIndex);
set(ylabel('ChiEKF','Interpreter','latex'));
set(xlabel('Time($s$)','Interpreter','latex'));
sgtitle("Bernoulli Zt index  (yellow - accept, blue - reject)",'Fontsize',10,'Interpreter','Latex');
pos = get(f6,'Position');
set(f6,'Units','Inches');
set(f6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

if (MC_times > 1)
    C = lines(4);
    items={'CHIEKFSEQ', 'VBEKF','BIGVBEKF','BIGVBEKFSEQ'};
    % state MC RMSE
    f7 = figure('Name','f7_state MC RMSE');
    subplot(231);boxplot([RMSE_CHIEKF_X(:,1), RMSE_VBEKF_X(:,1), RMSE_BIWVBEKF_X(:,1), RMSE_BIWVBEKFSEQ_X(:,1)],items,"Colors",C);
    set(ylabel(['Position $u(m)$'],'Interpreter','latex'));
    subplot(232);boxplot([RMSE_CHIEKF_X(:,2), RMSE_VBEKF_X(:,2), RMSE_BIWVBEKF_X(:,2), RMSE_BIWVBEKFSEQ_X(:,2)],items,"Colors",C);
    set(ylabel(['Velocity $\dot{u}(m/s)$'],'Interpreter','latex'));
    subplot(233);boxplot([RMSE_CHIEKF_X(:,3), RMSE_VBEKF_X(:,3), RMSE_BIWVBEKF_X(:,3), RMSE_BIWVBEKFSEQ_X(:,3)],items,"Colors",C);
    set(ylabel(['Position $v(m)$'],'Interpreter','latex'));
    subplot(234);boxplot([RMSE_CHIEKF_X(:,4), RMSE_VBEKF_X(:,4), RMSE_BIWVBEKF_X(:,4), RMSE_BIWVBEKFSEQ_X(:,4)],items,"Colors",C);
    set(ylabel(['Velocity $\dot{v}(m/s)$'],'Interpreter','latex'));
    subplot(235);boxplot([RMSE_CHIEKF_X(:,5), RMSE_VBEKF_X(:,5), RMSE_BIWVBEKF_X(:,5), RMSE_BIWVBEKFSEQ_X(:,5)],items,"Colors",C);
    set(ylabel(['Angular Rate $\omega(rad/s)$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo state RMSE Value",'Fontsize',10,'Interpreter','Latex');
    pos = get(f7,'Position');
    set(f7,'Units','Inches');
    set(f7,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

    % measurement noise statistics MC RMSE
    f8 = figure('Name','f8_measurement noise statistics MC RMSE');
    subplot(221);boxplot([RMSE_VBEKF_R(:,1), RMSE_BIWVBEKF_R(:,1), RMSE_BIWVBEKFSEQ_R(:,1)],items(2:end),"Colors",C);
    set(ylabel(['$R_1$'],'Interpreter','latex'));
    subplot(222);boxplot([RMSE_VBEKF_R(:,2), RMSE_BIWVBEKF_R(:,2), RMSE_BIWVBEKFSEQ_R(:,2)],items(2:end),"Colors",C);
    set(ylabel(['$R_2$'],'Interpreter','latex'));
    subplot(223);boxplot([RMSE_VBEKF_R(:,3), RMSE_BIWVBEKF_R(:,3), RMSE_BIWVBEKFSEQ_R(:,3)],items(2:end),"Colors",C);
    set(ylabel(['$R_3$'],'Interpreter','latex')); 
    subplot(224);boxplot([RMSE_VBEKF_R(:,4), RMSE_BIWVBEKF_R(:,4), RMSE_BIWVBEKFSEQ_R(:,4)],items(2:end),"Colors",C);
    set(ylabel(['$R_4$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo measurement noise RMSE Value",'Fontsize',10,'Interpreter','Latex');
    pos = get(f8,'Position');
    set(f8,'Units','Inches');
    set(f8,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
end

%% other funcs
function Xnext = Ft(Xcur, deltaT)
    F = [1, sin(Xcur(5)*deltaT)/Xcur(5),       0, (cos(Xcur(5)*deltaT) - 1)/Xcur(5), 0;
         0, cos(Xcur(5)*deltaT),               0, -sin(Xcur(5)*deltaT),              0;
         0, (1 - cos(Xcur(5)*deltaT))/Xcur(5), 1, sin(Xcur(5)*deltaT)/Xcur(5),       0;
         0, sin(Xcur(5)*deltaT),               0, cos(Xcur(5)*deltaT),               0;
         0, 0,                                 0, 0,                                 1;];

    Xnext = F * Xcur;
end

function Ycur = Ht(Xcur, Sensors)
    [r, ~] = size(Sensors);
    Ycur = zeros(r,1);
    for i = 1 : r 
        Ycur(i,:) = sqrt((Xcur(1) - Sensors(i, 1))^2 + (Xcur(3) - Sensors(i, 2))^2);
    end
end

function F = JabF(Xcur, deltaT)
    F = [1, sin(Xcur(5)*deltaT)/Xcur(5),       0, (cos(Xcur(5)*deltaT) - 1)/Xcur(5), (((deltaT*Xcur(5)*cos(Xcur(5)*deltaT)-sin(Xcur(5)*deltaT))*Xcur(2))-((deltaT*Xcur(5)*sin(Xcur(5)*deltaT)+cos(Xcur(5)*deltaT)-1)*Xcur(4)))/Xcur(5)^2;
         0, cos(Xcur(5)*deltaT),               0, -sin(Xcur(5)*deltaT),              -deltaT*(sin(Xcur(5)*deltaT)*Xcur(2)+ cos(Xcur(5)*deltaT)*Xcur(4));
         0, (1 - cos(Xcur(5)*deltaT))/Xcur(5), 1, sin(Xcur(5)*deltaT)/Xcur(5),       (((deltaT*sin(Xcur(5)*deltaT)*Xcur(5)-1+cos(Xcur(5)*deltaT)))*Xcur(2)+(deltaT*cos(Xcur(5)*deltaT))*Xcur(5)-sin(Xcur(5)*deltaT)*Xcur(4))/Xcur(5)^2;
         0, sin(Xcur(5)*deltaT),               0, cos(Xcur(5)*deltaT),               deltaT*(cos(Xcur(5)*deltaT)*Xcur(2)-sin(Xcur(5)*deltaT)*Xcur(4));
         0, 0,                                 0, 0,                                 1;];
end

function H = JabH(Xcur, Sensors)
    [r, ~] = size(Sensors);
    H = zeros(r, length(Xcur));
    for i = 1 : r 
        H(i,:) = [2*(Xcur(1) - Sensors(i,1))/sqrt((Xcur(1) - Sensors(i,1))^2 + (Xcur(3) - Sensors(i,2))^2), 0, 2*(Xcur(3) - Sensors(i,1))/sqrt((Xcur(1) - Sensors(i,1))^2 + (Xcur(3) - Sensors(i,2))^2), 0, 0];
    end 
end

%% END
