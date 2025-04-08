%% Inv-Gamma 5 dimension Bearing only tracking nonlinear model
% Yulu Zhong
% 5 state variables u, dot{u}, v, dot{v}, omega
% nonlinear observer
% 2024/3/27 southeast university
% [1] H. Wang, H. Li, J. Fang, and H. Wang, “Robust Gaussian Kalman Filter With Outlier Detection,” IEEE SIGNAL PROCESSING LETTERS, vol. 25, no. 8, pp. 1236–1240, Aug. 2018, doi: 10.1109/LSP.2018.2851156.
% [2] S. Sarkka, "Variational Bayesian Adaptation of Noise Covariances in Non-Linear Kalman Filtering,” arXiv, 2013.
% [3] S. Sarkka and A. Nummenmaa, "Recursive Noise Adaptive Kalman Filtering by Variational Bayesian Approximations,” IEEE T Automat Contr, vol. 54, no. 3, pp. 596–600, Mar. 2009, doi: 10.1109/tac.2008.2008348.
% [4] F. C. Zhu, Y. L. Huang, C. Xue, L. Mihaylova, and J. Chambers, “A Sliding Window Variational Outlier-Robust Kalman Filter Based on Student’s t-Noise Modeling,” IEEE T Aero Elec Sys, vol. 58, no. 5, pp. 4835–4849, Oct. 2022, doi: 10.1109/taes.2022.3164012.
% [5] Y. Huang, F. Zhu, G. Jia, and Y. Zhang, “A Slide Window Variational Adaptive Kalman Filter,” IEEE TRANSACTIONS ON CIRCUITS AND SYSTEMS II-EXPRESS BRIEFS, vol. 67, no. 12, pp. 3552–3556, Dec. 2020, doi: 10.1109/TCSII.2020.2995714. 
% [6] Y. Huang, Y. Zhang, Y. Zhao, P. Shi, and J. A. Chambers, “A novel outlier-robust kalman filtering framework based on statistical similarity measure,” IEEE Transactions on Automatic Control, vol. 66, no. 6, pp. 2677–2692, 2021, doi: 10.1109/TAC.2020.3011443.

%% Begin
clear;
close all;
clc;
% global parameters set
deltaT = 0.5;
stopT = 100;
outlierBegin = 40;
outlierEnd = 50;
susoutlierBegin = 60;
susoutlierEnd = 75.0;
MC_times = 500;
VB_D_Err_TH = 1e-10;
VB_it_Max = 10;

Time = zeros(1, fix(stopT/deltaT));

RMSE_EKF3p86_X = zeros(MC_times,5); % prealloc MC result for EKF
RMSE_EKF100_X = zeros(MC_times,5); % prealloc MC result for EKF
RMSE_VBEKF_X = zeros(MC_times,5); % prealloc MC result for VBEKF
RMSE_VBEKF_R = zeros(MC_times,4); % prealloc MC result for VBEKF
RMSE_BIGVBEKF_X = zeros(MC_times,5); % prealloc MC result for BIGVBEKF
RMSE_BIGVBEKF_R = zeros(MC_times,4); % prealloc MC result for BIGVBEKF
RMSE_BIGVBEKFSEQ_X = zeros(MC_times,5); % prealloc MC result for BIGVBEKF
RMSE_BIGVBEKFSEQ_R = zeros(MC_times,4); % prealloc MC result for BIGVBEKF
RMSE_NEWAPRIVBKF_X = zeros(MC_times, 5); % prealloc MC result for resNew_aprivbkf
RMSE_NEWAPRIVBKF_R = zeros(MC_times,4); % prealloc MC result for resNew_aprivbkf
RMSE_SWRKF_X = zeros(MC_times, 5); % prealloc MC result for resSwrkf
RMSE_SWRKF_R = zeros(MC_times,4); % prealloc MC result for resSwrkf
RMSE_SSMKF_X = zeros(MC_times, 5); % prealloc MC result for resSsmkf
RMSE_SSMKF_R = zeros(MC_times, 5); % prealloc MC result for resSsmkf
RMSE_RSTKF_X = zeros(MC_times,4); % prealloc MC result for RSTKF
RMSE_RSTKF_R = zeros(MC_times,4); % prealloc MC result for RSTKF
RMSE_MCKF_X = zeros(MC_times, 4); % prealloc MC result for resMCKF
RMSE_MCKF_R = zeros(MC_times, 4); % prealloc MC result for resMCKF
MCCHIKFSEQ3p86Time = zeros(MC_times,1); % prealloc MC result for chi KF SEQ time
MCCHIKFSEQ100Time = zeros(MC_times,1); % prealloc MC result for chi KF SEQ time
MCVBKFTime = zeros(MC_times,1); % prealloc MC result for VBKF
MCRSTKFTime = zeros(MC_times,1); % prealloc MC result for VBKFSEQ
MCNew_aprivbkfTime = zeros(MC_times,1); % prealloc MC result for resNew_aprivbkf
MCSwrkfTime = zeros(MC_times,1); % prealloc MC result for resSwrkf
MCSsmkfTime = zeros(MC_times,1); % prealloc MC result for resSsmkf
MCMckfTime = zeros(MC_times,1); % prealloc MC result for resMCKF
MCBIGVBKFTime = zeros(MC_times,1); % prealloc MC result for BIGVBEKF
MCBIGVBKFSEQTime = zeros(MC_times,1); % prealloc MC result for BIGVBEKFSEQ

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


abnomal_Lambda = 0.5;

abnomal_Alpha = 400;

sustainMu=400;

X0 = [-0, 5, 0, 5, 0.1]';

R = 10^2 .* ones(4, fix(stopT/deltaT)) - 5^2.* cos(0.02*(0:1:fix(stopT/deltaT)-1));
outlierGm_Mu = zeros(2,4);

nominal_R = diag(R(:,1));
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
%% chiEKF - 3.86
    resEKF3p86_X = zeros(5, fix(stopT/deltaT));
    resSEQEKF3p86CHI = zeros(4, fix(stopT/deltaT));
    resSEQEKFCHI3p86Index = zeros(4, fix(stopT/deltaT));
    resSEQKFCHI3p86Time = zeros(1, fix(stopT/deltaT));
    EKF3p86_Px = filter_P0;
    EKF3p86_X = filter_X0;
    EKF3p86_R = nominal_R;
    SEQEKFCHI3p86th = 3.86;
    resEKF3p86_RMSE = zeros(1, fix(stopT/deltaT));
    k=1;
    for t = 0 : deltaT : stopT-deltaT
        tic;
        % time update
        F = JabF(EKF3p86_X, deltaT);
        EKF3p86_X = Ft(EKF3p86_X, deltaT);
        EKF3p86_Px = F*EKF3p86_Px*F'+ Q;
        % measurement update
        H = JabH(EKF3p86_X, SensorsPoint);
        EKF3p86_Hk = H;
        SEQEKF3p86Hn = size(EKF3p86_Hk,1);
        EKF3p86_R = diag(R(:,k));
        for i = 1:SEQEKF3p86Hn
            EKF_Sk = EKF3p86_Hk(i,:)*EKF3p86_Px*EKF3p86_Hk(i,:)'+ EKF3p86_R(i,i);
            SEQEKFCHI3p86 = (Ypse(i,k) - Ht(EKF3p86_X, SensorsPoint(i,:)))'/EKF_Sk*(Ypse(i,k) - Ht(EKF3p86_X, SensorsPoint(i,:)));
            resSEQEKF3p86CHI(i,k) = SEQEKFCHI3p86;
            if (SEQEKFCHI3p86 < SEQEKFCHI3p86th)
                resSEQEKFCHI3p86Index(i,k) = 1;
                EKF_Kk = EKF3p86_Px*EKF3p86_Hk(i,:)'/EKF_Sk;
                EKF3p86_X = EKF3p86_X + EKF_Kk*(Ypse(i,k) - Ht(EKF3p86_X, SensorsPoint(i,:)));
                EKF3p86_Px = EKF3p86_Px - EKF_Kk*EKF_Sk*EKF_Kk';
            end
        end
        tSEQEKFCHI3p86End = toc;
        % res
        resEKF3p86_X(:,k) = EKF3p86_X;
        resEKF3p86_RMSE(:,k) = sqrt(trace(EKF3p86_Px));
        resSEQKFCHI3p86Time(:,k) = tSEQEKFCHI3p86End;
        k=k+1;
    end
%% chiEKF - 100
    resEKF100_X = zeros(5, fix(stopT/deltaT));
    resSEQEKF100CHI = zeros(4, fix(stopT/deltaT));
    resSEQEKFCHI100Index = zeros(4, fix(stopT/deltaT));
    resSEQKFCHI100Time = zeros(1, fix(stopT/deltaT));
    EKF100_Px = filter_P0;
    EKF100_X = filter_X0;
    EKF100_R = nominal_R;
    SEQEKFCHI100th = 100;
    resEKF100_RMSE = zeros(1, fix(stopT/deltaT));
    k=1;
    for t = 0 : deltaT : stopT-deltaT
        tic;
        % time update
        F = JabF(EKF100_X, deltaT);
        EKF100_X = Ft(EKF100_X, deltaT);
        EKF100_Px = F*EKF100_Px*F'+ Q;
        % measurement update
        H = JabH(EKF100_X, SensorsPoint);
        EKF100_Hk = H;
        SEQEKF100Hn = size(EKF100_Hk,1);
        EKF100_R = diag(R(:,k));
        for i = 1:SEQEKF100Hn
            EKF_Sk = EKF100_Hk(i,:)*EKF100_Px*EKF100_Hk(i,:)'+ EKF100_R(i,i);
            SEQEKFCHI100 = (Ypse(i,k) - Ht(EKF100_X, SensorsPoint(i,:)))'/EKF_Sk*(Ypse(i,k) - Ht(EKF100_X, SensorsPoint(i,:)));
            resSEQEKF100CHI(i,k) = SEQEKFCHI100;
            if (SEQEKFCHI100 < SEQEKFCHI100th)
                resSEQEKFCHI100Index(i,k) = 1;
                EKF_Kk = EKF100_Px*EKF100_Hk(i,:)'/EKF_Sk;
                EKF100_X = EKF100_X + EKF_Kk*(Ypse(i,k) - Ht(EKF100_X, SensorsPoint(i,:)));
                EKF100_Px = EKF100_Px - EKF_Kk*EKF_Sk*EKF_Kk';
            end
        end
        tSEQEKFCHI100End = toc;
        % res
        resEKF100_X(:,k) = EKF100_X;
        resEKF100_RMSE(:,k) = sqrt(trace(EKF100_Px));
        resSEQKFCHI100Time(:,k) = tSEQEKFCHI100End;
        k=k+1;
    end

%% VBEKF
    resVBEKF_X_d = zeros(5, fix(stopT/deltaT));
    resVBEKF_R_d = zeros(4, fix(stopT/deltaT));
    resVBEKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resVBEKF_it_d = zeros(1, fix(stopT/deltaT));
    resVBKFTime = zeros(1, fix(stopT/deltaT));
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
        tic;
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
        tVBKFEnd = toc;
        resVBEKF_it_d(:,k) = i;
        %res
        resVBEKF_X_d(:,k) = VBEKF_X_d;
        resVBEKF_R_d(:,k) = diag(VBEKF_R_d);
        resVBEKF_RMSE_d(:,k) = sqrt(trace(VBEKF_Px_d));
        resVBKFTime(:,k) = tVBKFEnd;
        k=k+1;
    end   
%% A Slide Window Variational Adaptive Kalman Filter - new_aprivbkf
     % res 
     resNew_aprivbkf_comp_X = zeros(5, fix(stopT / deltaT));
     resNew_aprivbkf_comp_R = zeros(4, fix(stopT / deltaT));
     resNew_aprivbkf_comp_RMSE = zeros(1, fix(stopT / deltaT));
     resNew_aprivbkfTime = zeros(1, fix(stopT/deltaT));
     % filter parameters set
     new_xapriv=filter_X0;
     new_Papriv=filter_P0;
     new_xapriv_A=[];
     new_Papriv_A=[];
     new_yapriv=0;
     new_Yapriv=zeros(5);
     new_uapriv=0;
     new_Uapriv=zeros(4);
     new_Qapriv=Q;
     new_Rapriv=diag(R(:,1));
     new_Lapriv=5;
     rou=1-exp(-4);
     zA=[];
     k=1;
     for t = 0:deltaT:stopT-deltaT
        tic;
        % 
        if t<=(new_Lapriv+1)*deltaT
           zA=[zA Ypse(:,k)];
        else
           zA=[zA(:,2:end) Ypse(:,k)];
        end
        [new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,new_Qapriv,new_Rapriv,new_Ppapriv]=...
            new_aprivbkf_comp(new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,zA,new_Qapriv,new_Rapriv,rou,new_Lapriv,k,deltaT);
        tNew_aprivbkfEnd = toc;
        % res
        resNew_aprivbkf_comp_X(:,k) = new_xapriv;
        resNew_aprivbkf_comp_R(:,k) = diag(new_Rapriv);
        resNew_aprivbkf_comp_RMSE(:,k) = sqrt(trace(new_Papriv));
        resNew_aprivbkfTime(:,k) = tNew_aprivbkfEnd;
        k=k+1;
     end

%% A Sliding Window Variational Outlier-Robust Kalman Filter Based on Student s t-Noise Modeling - swrkf
     % res       
     resSwrkf_X = zeros(5, fix(stopT / deltaT));
     resSwrkf_R = zeros(4, fix(stopT / deltaT));
     resSwrkf_RMSE = zeros(1, fix(stopT / deltaT));
     resSwrkfTime = zeros(1, fix(stopT / deltaT));
     % filter parameters set
     tao_R=5;
     tao_Q=5;
     epsilon=VB_D_Err_TH;
     rou=1-exp(-4);
     omega=5;
     nu=5;
     L=10;
     zA=[];
     xsw=filter_X0;
     Psw=filter_P0;
     xsw_A=xsw;
     Psw_A=Psw;
     ysw_A=tao_Q;
     Ysw_A=tao_Q*Q;
     usw_A=tao_R;
     Usw_A=tao_R*diag(R(:,1));
     Qsw=Q;
     Rsw=nominal_R;
     ksisw=[];
     lambdasw=[];
     k=1;
     for t = 0:deltaT:stopT-deltaT
        tic;
        %
        if t<=(L*deltaT)
           zA=[zA Ypse(:,k)];
        else
           zA=[zA(:,2:end) Ypse(:,k)];
        end
        [xsw,Psw,xsw_A,Psw_A,xswnkNA,PswnkNA,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,ksisw,lambdasw,Nsw,M]=...
            swrkf_comp(xsw_A,Psw_A,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,zA,ksisw,lambdasw,omega,nu,rou,L,k,VB_it_Max,epsilon,deltaT);
        tSwrkfEnd = toc;
        %res
        resSwrkf_X(:,k) = xsw;
        resSwrkf_R(:,k) = diag(Rsw);
        resSwrkf_RMSE(:,k) = sqrt(trace(Psw));
        resSwrkfTime(:,k) = tSwrkfEnd;
        k=k+1;
     end
%% A novel outlier-robust kalman filtering framework based on statistical similarity measure - ssmkf
    % res  
    resSsmkf_X = zeros(5, fix(stopT / deltaT));
    resSsmkf_R = zeros(4, fix(stopT / deltaT));
    resSsmkf_RMSE = zeros(1, fix(stopT / deltaT));
    resSsmkfTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    xssm_13=filter_X0;
    Pssm_13=filter_P0;
    N=VB_it_Max;              %%%%%%最大变分迭代次数
    gama=1.345;        %%%%%%调节参数
    sigma=5;           %%%%%%高斯核带宽
    v_sq=5;            %%%%%%自由度参数
    tao_P=2;
    tao_R=2;
    epsilon=VB_D_Err_TH;
    Q0 = Q;
    R0 = nominal_R;
    k=1;
    for t = 0:deltaT:stopT-deltaT
        tic;
        %
        z = Ypse(:,k);
        [xssm_13,Pssm_13,DR]=ssmkf_comp(xssm_13,Pssm_13,z,Q0,R0,sigma,v_sq,N,epsilon,3,deltaT);
        tSsmkfEnd = toc;
        % res
        resSsmkf_X(:,k) = xssm_13;
        resSsmkf_R(:,k) = diag(DR);
        resSsmkf_RMSE(:,k) = sqrt(trace(Pssm_13));
        resSsmkfTime(:,k) = tSsmkfEnd;
        k = k + 1;
    end
 %% RSTKF
    % res
    resRSTEKF_X = zeros(5, fix(stopT / deltaT));
    resRSTEKF_RMSE = zeros(1, fix(stopT / deltaT));
    resRSTEKF_R = zeros(4, fix(stopT / deltaT));
    resRSTKFTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    rstekf.xk= filter_X0;
    rstekf.Pxk = filter_P0;
    rstekf.Gammak = 1;   rstekf.Qk = Q;  rstekf.measIter = VB_it_Max;
    [rstekf.m, rstekf.n] = size(H);
    rstekf.Rk = nominal_R;
    k=1;
    for t = 0:deltaT:stopT-deltaT
       tic;
       rstekf = akfupdate_comp(rstekf, Ypse(:,k), 'B', 'RSTKF', 5, VB_D_Err_TH,deltaT);
       tRstkfEnd = toc;
       % res
       resRSTEKF_X(:,k) = rstekf.xk;
       resRSTEKF_R(:,k) = diag(rstekf.res);
       resRSTEKF_RMSE(:,k) = sqrt(trace(rstekf.Pxk));
       resRSTKFTime(:,k) = tRstkfEnd;
       k = k + 1;
    end
 %% maximum correntropy Kalman filter
    % res
    resMCEKF_X = zeros(5, fix(stopT / deltaT));
    resMCEKF_R = zeros(4, fix(stopT / deltaT));
    resMCEKF_RMSE = zeros(1, fix(stopT / deltaT));
    resMCKFTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    mcekf.xk= filter_X0;
    mcekf.Pxk = filter_P0;
    mcekf.Gammak = 1;   mcekf.Qk = Q;  mcekf.measIter = VB_it_Max;
    mcekf.Phikk_1 = F; [mcekf.m, mcekf.n] = size(H);
    mcekf.Rk = nominal_R;
    k=1;
    for t = 0:deltaT:stopT-deltaT
%        tMckfStart = tic;
       tic;
       mcekf = akfupdate_comp(mcekf, Ypse(:,k), 'B', 'MCKF',  5, VB_D_Err_TH,deltaT);
       tMckfEnd = toc;
       % res
       resMCEKF_X(:,k) = mcekf.xk;
       resMCEKF_R(:,k) = diag(mcekf.res);
       resMCEKF_RMSE(:,k) = sqrt(trace(mcekf.Pxk));
       resMCKFTime(:,k) = tMckfEnd;
       k = k + 1;
    end
%% BIGVBEKF (bern inv Gamma) - MINE
    resBIGVBEKF_X_d = zeros(5, fix(stopT/deltaT));
    resBIGVBEKF_R_d = zeros(4, fix(stopT/deltaT));
    resBIGVBEKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resBIGVBEKF_it_d = zeros(1, fix(stopT/deltaT));
    resBIGVBZt = zeros(4, fix(stopT/deltaT));
    resBIGVBKFTime = zeros(1, fix(stopT/deltaT));
    BIGVBEKF_X_d = filter_X0;
    BIGVBEKF_Px_d = filter_P0;  
    BIGVBEKF_BerZt_d = 1; % Bernoulli param. 
    BIGVBEKF_BerZt_TH = 0.1;
    BIGVBEKF_BetaE0_d = 0.9; BIGVBEKF_BetaF0_d = 0.1; % Beta params.
%     BIWVBEKF_BerZt_TH = 0.1;
%     BIWVBEKF_BetaE0_d = 0.8; BIWVBEKF_BetaF0_d = 0.2; % Beta params.
    BIGVBEKF_IGa1_d = 1; BIGVBEKF_IGb1_d = R(1,1);%BIWVBEKF_IGb1_d = 50^2; % Inv-Gamma param.
    BIGVBEKF_IGa2_d = 1; BIGVBEKF_IGb2_d = R(2,1);%BIWVBEKF_IGb2_d = 50^2; % Inv-Gamma param.
    BIGVBEKF_IGa3_d = 1; BIGVBEKF_IGb3_d = R(3,1);%BIWVBEKF_IGb3_d = 50^2; % Inv-Gamma param.
    BIGVBEKF_IGa4_d = 1; BIGVBEKF_IGb4_d = R(4,1);%BIWVBEKF_IGb4_d = 50^2; % Inv-Gamma param.
%     BIWVBEKF_it_max_d = 20; % iteration times
    BIGVBEKF_it_max_d = VB_it_Max;
    BIGVBEKF_E_d = VB_D_Err_TH; % error threshold
    BIGVBEKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0 : deltaT : stopT - deltaT
        tic;
        % time update
        F = JabF(BIGVBEKF_X_d, deltaT); 
        BIGVBEKF_X_d = Ft(BIGVBEKF_X_d, deltaT); 
        BIGVBEKF_Px_d = F * BIGVBEKF_Px_d * F' + Q; 
        BIGVBEKF_IGa1_d = BIGVBEKF_pho_d * BIGVBEKF_IGa1_d; BIGVBEKF_IGa2_d = BIGVBEKF_pho_d * BIGVBEKF_IGa2_d; % heuristic dynamic
        BIGVBEKF_IGb1_d = BIGVBEKF_pho_d * BIGVBEKF_IGb1_d; BIGVBEKF_IGb2_d = BIGVBEKF_pho_d * BIGVBEKF_IGb2_d; % heuristic dynamic
        BIGVBEKF_IGa3_d = BIGVBEKF_pho_d * BIGVBEKF_IGa3_d; BIGVBEKF_IGa4_d = BIGVBEKF_pho_d * BIGVBEKF_IGa4_d; % heuristic dynamic
        BIGVBEKF_IGb3_d = BIGVBEKF_pho_d * BIGVBEKF_IGb3_d; BIGVBEKF_IGb4_d = BIGVBEKF_pho_d * BIGVBEKF_IGb4_d; % heuristic dynamic
        % measurement update
        H = JabH(BIGVBEKF_X_d, SensorsPoint);
%         BIWVBEKF_Hk_d = H;
        BIGVBEKF_IGa1_d = 0.5 + BIGVBEKF_IGa1_d; % VB step1
        BIGVBEKF_IGa2_d = 0.5 + BIGVBEKF_IGa2_d; 
        BIGVBEKF_IGa3_d = 0.5 + BIGVBEKF_IGa3_d; 
        BIGVBEKF_IGa4_d = 0.5 + BIGVBEKF_IGa4_d; 
        tmpBIGVBEKF_BerZt1_d = BIGVBEKF_BerZt_d; 
        tmpBIGVBEKF_BerZt2_d = BIGVBEKF_BerZt_d;
        tmpBIGVBEKF_BerZt3_d = BIGVBEKF_BerZt_d;
        tmpBIGVBEKF_BerZt4_d = BIGVBEKF_BerZt_d;
        BIGVBEKF_BetaEt1_d = BIGVBEKF_BetaE0_d; BIGVBEKF_BetaFt1_d = BIGVBEKF_BetaF0_d;
        BIGVBEKF_BetaEt2_d = BIGVBEKF_BetaE0_d; BIGVBEKF_BetaFt2_d = BIGVBEKF_BetaF0_d;
        BIGVBEKF_BetaEt3_d = BIGVBEKF_BetaE0_d; BIGVBEKF_BetaFt3_d = BIGVBEKF_BetaF0_d;
        BIGVBEKF_BetaEt4_d = BIGVBEKF_BetaE0_d; BIGVBEKF_BetaFt4_d = BIGVBEKF_BetaF0_d;
        tmpBIGVBEKF_X_d = BIGVBEKF_X_d; tmpBIGVBEKF_IGb1_d = BIGVBEKF_IGb1_d; 
        tmpBIGVBEKF_Px_d = BIGVBEKF_Px_d; tmpBIGVBEKF_IGb2_d = BIGVBEKF_IGb2_d;
        tmpBIGVBEKF_IGb3_d = BIGVBEKF_IGb3_d; tmpBIGVBEKF_IGb4_d = BIGVBEKF_IGb4_d;
        for i = 1 : BIGVBEKF_it_max_d % VB iterate dynamic times
%             H = JabH(BIWVBEKF_X_d, SensorsPoint);
            BIGVBEKF_X_d_pre = BIGVBEKF_X_d;
            if ((tmpBIGVBEKF_BerZt1_d > BIGVBEKF_BerZt_TH) || (tmpBIGVBEKF_BerZt2_d > BIGVBEKF_BerZt_TH) || (tmpBIGVBEKF_BerZt3_d > BIGVBEKF_BerZt_TH) || (tmpBIGVBEKF_BerZt4_d > BIGVBEKF_BerZt_TH))
                BIGVBEKF_Hk_d = []; BIGVBEKF_R_d = []; BIGVBEKFSensorsPoint = []; YBIGVBEKF = [];
                if tmpBIGVBEKF_BerZt1_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Hk_d = [BIGVBEKF_Hk_d; H(1,:)]; 
                    YBIGVBEKF = [YBIGVBEKF; Ypse(1,k)]; 
                    BIGVBEKFSensorsPoint = [BIGVBEKFSensorsPoint; SensorsPoint(1,:)]; 
                    BIGVBEKF_R_d = [BIGVBEKF_R_d, BIGVBEKF_IGb1_d / BIGVBEKF_IGa1_d / tmpBIGVBEKF_BerZt1_d]; 
                end
                if tmpBIGVBEKF_BerZt2_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Hk_d = [BIGVBEKF_Hk_d; H(2,:)]; 
                    YBIGVBEKF = [YBIGVBEKF; Ypse(2,k)]; 
                    BIGVBEKFSensorsPoint = [BIGVBEKFSensorsPoint; SensorsPoint(2,:)]; 
                    BIGVBEKF_R_d = [BIGVBEKF_R_d, BIGVBEKF_IGb2_d / BIGVBEKF_IGa2_d / tmpBIGVBEKF_BerZt2_d]; 
                end
                if tmpBIGVBEKF_BerZt3_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Hk_d = [BIGVBEKF_Hk_d; H(3,:)];
                    YBIGVBEKF = [YBIGVBEKF; Ypse(3,k)];
                    BIGVBEKFSensorsPoint = [BIGVBEKFSensorsPoint; SensorsPoint(3,:)];
                    BIGVBEKF_R_d = [BIGVBEKF_R_d, BIGVBEKF_IGb3_d / BIGVBEKF_IGa3_d / tmpBIGVBEKF_BerZt3_d];
                end
                if tmpBIGVBEKF_BerZt4_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Hk_d = [BIGVBEKF_Hk_d; H(4,:)];
                    YBIGVBEKF = [YBIGVBEKF; Ypse(4,k)];
                    BIGVBEKFSensorsPoint = [BIGVBEKFSensorsPoint; SensorsPoint(4,:)];
                    BIGVBEKF_R_d = [BIGVBEKF_R_d, BIGVBEKF_IGb4_d / BIGVBEKF_IGa4_d / tmpBIGVBEKF_BerZt4_d];
                end
                BIGVBEKF_R_d = diag(BIGVBEKF_R_d);
                BIGVBEKF_Sk_d = BIGVBEKF_Hk_d * tmpBIGVBEKF_Px_d * BIGVBEKF_Hk_d'+ BIGVBEKF_R_d; 
                BIGVBEKF_Kk_d = tmpBIGVBEKF_Px_d * BIGVBEKF_Hk_d' / BIGVBEKF_Sk_d; 
                BIGVBEKF_X_d = tmpBIGVBEKF_X_d + BIGVBEKF_Kk_d * (YBIGVBEKF - Ht(tmpBIGVBEKF_X_d, BIGVBEKFSensorsPoint)); 
                BIGVBEKF_Px_d = tmpBIGVBEKF_Px_d - BIGVBEKF_Kk_d * BIGVBEKF_Sk_d * BIGVBEKF_Kk_d'; % VB-E step done 
                if tmpBIGVBEKF_BerZt1_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Bt = (Ypse(1,k) - Ht(BIGVBEKF_X_d, SensorsPoint(1,:))) * (Ypse(1,k) - Ht(BIGVBEKF_X_d, SensorsPoint(1,:)))' +  H(1,:) * BIGVBEKF_Px_d * H(1,:)';
                    tmpBIGVBEKF_BerZt1_d = exp(psi(BIGVBEKF_BetaEt1_d) - psi(BIGVBEKF_BetaEt1_d + BIGVBEKF_BetaFt1_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa1_d / BIGVBEKF_IGb1_d)) / ...
                             (exp(psi(BIGVBEKF_BetaEt1_d) - psi(BIGVBEKF_BetaEt1_d + BIGVBEKF_BetaFt1_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa1_d / BIGVBEKF_IGb1_d)) + ...
                                exp(psi(BIGVBEKF_BetaFt1_d ) - psi(BIGVBEKF_BetaEt1_d + BIGVBEKF_BetaFt1_d )));
                    BIGVBEKF_IGb1_d = tmpBIGVBEKF_IGb1_d +  0.5 * tmpBIGVBEKF_BerZt1_d * (Ypse(1,k) - Ht(BIGVBEKF_X_d,  SensorsPoint(1,:))).^2 + ...
                        0.5 * tmpBIGVBEKF_BerZt1_d * diag((H(1,:) * BIGVBEKF_Px_d * H(1,:)')); 
                    BIGVBEKF_BetaEt1_d = BIGVBEKF_BetaE0_d + tmpBIGVBEKF_BerZt1_d; 
                    BIGVBEKF_BetaFt1_d = BIGVBEKF_BetaF0_d + 1 - tmpBIGVBEKF_BerZt1_d; % VB-M step done 
                else
                    BIGVBEKF_IGb1_d = tmpBIGVBEKF_IGb1_d; 
                end
                if tmpBIGVBEKF_BerZt2_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Bt = (Ypse(2,k) - Ht(BIGVBEKF_X_d, SensorsPoint(2,:))) * (Ypse(2,k) - Ht(BIGVBEKF_X_d, SensorsPoint(2,:)))' +  H(2,:) * BIGVBEKF_Px_d * H(2,:)';
                    tmpBIGVBEKF_BerZt2_d = exp(psi(BIGVBEKF_BetaEt2_d) - psi(BIGVBEKF_BetaEt2_d + BIGVBEKF_BetaFt2_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa2_d / BIGVBEKF_IGb2_d)) / ...
                             (exp(psi(BIGVBEKF_BetaEt2_d) - psi(BIGVBEKF_BetaEt2_d + BIGVBEKF_BetaFt2_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa2_d / BIGVBEKF_IGb2_d)) + ...
                                exp(psi(BIGVBEKF_BetaFt2_d ) - psi(BIGVBEKF_BetaEt2_d + BIGVBEKF_BetaFt2_d )));
                    BIGVBEKF_IGb2_d = tmpBIGVBEKF_IGb2_d +  0.5 * tmpBIGVBEKF_BerZt2_d * (Ypse(2,k) - Ht(BIGVBEKF_X_d,  SensorsPoint(2,:))).^2 + ...
                        0.5 * tmpBIGVBEKF_BerZt2_d * diag((H(2,:) * BIGVBEKF_Px_d * H(2,:)')); 
                    BIGVBEKF_BetaEt2_d = BIGVBEKF_BetaE0_d + tmpBIGVBEKF_BerZt2_d;
                    BIGVBEKF_BetaFt2_d = BIGVBEKF_BetaF0_d + 1 - tmpBIGVBEKF_BerZt2_d; % VB-M step done 
                else
                    BIGVBEKF_IGb2_d = tmpBIGVBEKF_IGb2_d; 
                end
                if tmpBIGVBEKF_BerZt3_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Bt = (Ypse(3,k) - Ht(BIGVBEKF_X_d, SensorsPoint(3,:))) * (Ypse(3,k) - Ht(BIGVBEKF_X_d, SensorsPoint(3,:)))' +  H(3,:) * BIGVBEKF_Px_d * H(3,:)';
                    tmpBIGVBEKF_BerZt3_d = exp(psi(BIGVBEKF_BetaEt3_d) - psi(BIGVBEKF_BetaEt3_d + BIGVBEKF_BetaFt3_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa3_d / BIGVBEKF_IGb3_d)) / ...
                             (exp(psi(BIGVBEKF_BetaEt3_d) - psi(BIGVBEKF_BetaEt3_d + BIGVBEKF_BetaFt3_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa3_d / BIGVBEKF_IGb3_d)) + ...
                                exp(psi(BIGVBEKF_BetaFt3_d ) - psi(BIGVBEKF_BetaEt3_d + BIGVBEKF_BetaFt3_d )));
                    BIGVBEKF_IGb3_d = tmpBIGVBEKF_IGb3_d +  0.5 * tmpBIGVBEKF_BerZt3_d * (Ypse(3,k) - Ht(BIGVBEKF_X_d,  SensorsPoint(3,:))).^2 + ...
                        0.5 * tmpBIGVBEKF_BerZt3_d * diag((H(3,:) * BIGVBEKF_Px_d * H(3,:)')); 
                    BIGVBEKF_BetaEt3_d = BIGVBEKF_BetaE0_d + tmpBIGVBEKF_BerZt3_d;
                    BIGVBEKF_BetaFt3_d = BIGVBEKF_BetaF0_d + 1 - tmpBIGVBEKF_BerZt3_d; % VB-M step done 
                else
                    BIGVBEKF_IGb3_d = tmpBIGVBEKF_IGb3_d; 
                end
                if tmpBIGVBEKF_BerZt4_d > BIGVBEKF_BerZt_TH 
                    BIGVBEKF_Bt = (Ypse(4,k) - Ht(BIGVBEKF_X_d, SensorsPoint(4,:))) * (Ypse(4,k) - Ht(BIGVBEKF_X_d, SensorsPoint(4,:)))' +  H(4,:) * BIGVBEKF_Px_d * H(4,:)';
                    tmpBIGVBEKF_BerZt4_d = exp(psi(BIGVBEKF_BetaEt4_d) - psi(BIGVBEKF_BetaEt4_d + BIGVBEKF_BetaFt4_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa4_d / BIGVBEKF_IGb4_d)) / ...
                             (exp(psi(BIGVBEKF_BetaEt4_d) - psi(BIGVBEKF_BetaEt4_d + BIGVBEKF_BetaFt4_d ) - 0.5 * trace(BIGVBEKF_Bt * BIGVBEKF_IGa4_d / BIGVBEKF_IGb4_d)) + ...
                                exp(psi(BIGVBEKF_BetaFt4_d ) - psi(BIGVBEKF_BetaEt4_d + BIGVBEKF_BetaFt4_d )));
                    BIGVBEKF_IGb4_d = tmpBIGVBEKF_IGb4_d +  0.5 * tmpBIGVBEKF_BerZt4_d * (Ypse(4,k) - Ht(BIGVBEKF_X_d,  SensorsPoint(4,:))).^2 + ...
                        0.5 * tmpBIGVBEKF_BerZt4_d * diag((H(4,:) * BIGVBEKF_Px_d * H(4,:)')); 
                    BIGVBEKF_BetaEt4_d = BIGVBEKF_BetaE0_d + tmpBIGVBEKF_BerZt4_d;
                    BIGVBEKF_BetaFt4_d = BIGVBEKF_BetaF0_d + 1 - tmpBIGVBEKF_BerZt4_d; % VB-M step done 
                else 
                    BIGVBEKF_IGb4_d = tmpBIGVBEKF_IGb4_d; 
                end
                if(norm(BIGVBEKF_X_d - BIGVBEKF_X_d_pre) < BIGVBEKF_E_d)
                    break;
                end
            else
                BIGVBEKF_X_d = tmpBIGVBEKF_X_d; 
                BIGVBEKF_Px_d = tmpBIGVBEKF_Px_d; 
                BIGVBEKF_IGb1_d = tmpBIGVBEKF_IGb1_d; 
                BIGVBEKF_IGb2_d = tmpBIGVBEKF_IGb2_d; 
                BIGVBEKF_IGb3_d = tmpBIGVBEKF_IGb3_d; 
                BIGVBEKF_IGb4_d = tmpBIGVBEKF_IGb4_d; 
                break;
            end
        end
        if tmpBIGVBEKF_BerZt1_d <= BIGVBEKF_BerZt_TH 
            BIGVBEKF_IGa1_d =  BIGVBEKF_IGa1_d - 0.5; 
        end
        if tmpBIGVBEKF_BerZt2_d <= BIGVBEKF_BerZt_TH 
            BIGVBEKF_IGa2_d =  BIGVBEKF_IGa2_d - 0.5; 
        end
        if tmpBIGVBEKF_BerZt3_d <= BIGVBEKF_BerZt_TH 
            BIGVBEKF_IGa3_d =  BIGVBEKF_IGa3_d - 0.5; 
        end
        if tmpBIGVBEKF_BerZt4_d <= BIGVBEKF_BerZt_TH 
            BIGVBEKF_IGa4_d =  BIGVBEKF_IGa4_d - 0.5; 
        end
        BIGVBEKF_R_d = diag([BIGVBEKF_IGb1_d / BIGVBEKF_IGa1_d, BIGVBEKF_IGb2_d / BIGVBEKF_IGa2_d,...
            BIGVBEKF_IGb3_d / BIGVBEKF_IGa3_d, BIGVBEKF_IGb4_d / BIGVBEKF_IGa4_d]); 
        tBIGVBKFEnd = toc;
        resBIGVBEKF_it_d(:,k) = i;
        %res
        resBIGVBEKF_X_d(:,k) = BIGVBEKF_X_d;
        resBIGVBEKF_R_d(:,k) = diag(BIGVBEKF_R_d);
        resBIGVBZt(:,k) = [tmpBIGVBEKF_BerZt1_d; tmpBIGVBEKF_BerZt2_d;...
             tmpBIGVBEKF_BerZt3_d; tmpBIGVBEKF_BerZt4_d];
        resBIGVBEKF_RMSE_d(:,k) = sqrt(trace(BIGVBEKF_Px_d));
        resBIGVBKFTime(:,k) = tBIGVBKFEnd;
        k=k+1;
    end   

%% BIGVBEKF (bern inv Gamma) SEQ - MINE
    resBIGVBEKFSEQ_X_d = zeros(5, fix(stopT / deltaT));
    resBIGVBEKFSEQ_R_d = zeros(4, fix(stopT / deltaT));
    resBIGVBEKFSEQ_RMSE_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_1st_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_2nd_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_3rd_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_4th_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBSEQZt = zeros(4, fix(stopT / deltaT));
    resBIGVBKFSEQTime = zeros(1, fix(stopT / deltaT));
    BIGVBEKFSEQ_X_d = filter_X0;
    BIGVBEKFSEQ_Px_d = filter_P0;  
    BIGVBEKFSEQ_BerZt_d = 1; % Bernoulli param. 
    BIGVBEKFSEQ_BerZt_TH = 0.1;
    BIGVBEKFSEQ_BetaE0_d = 0.9; BIGVBEKFSEQ_BetaF0_d = 0.1; % Beta params.
%     BIWVBEKFSEQ_BerZt_TH = 0.1;
%     BIWVBEKFSEQ_BetaE0_d = 0.8; BIWVBEKFSEQ_BetaF0_d = 0.2; % Beta params.
    BIGVBEKFSEQ_IGa1_d = 1; BIGVBEKFSEQ_IGb1_d = R(1,1);%BIWVBEKFSEQ_IGb1_d = 50^2; % Inv-Gamma param.
    BIGVBEKFSEQ_IGa2_d = 1; BIGVBEKFSEQ_IGb2_d = R(2,1);%BIWVBEKFSEQ_IGb2_d = 50^2; % Inv-Gamma param.
    BIGVBEKFSEQ_IGa3_d = 1; BIGVBEKFSEQ_IGb3_d = R(3,1);%BIWVBEKFSEQ_IGb3_d = 50^2; % Inv-Gamma param.
    BIGVBEKFSEQ_IGa4_d = 1; BIGVBEKFSEQ_IGb4_d = R(4,1);%BIWVBEKFSEQ_IGb4_d = 50^2; % Inv-Gamma param.
%     BIWVBEKFSEQ_it_max_d = 20; % iteration times
    BIGVBEKFSEQ_it_max_d = VB_it_Max;
    BIGVBEKFSEQ_E_d = VB_D_Err_TH; % error threshold
    BIGVBEKFSEQ_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        tic;
        % time update
        F = JabF(BIGVBEKFSEQ_X_d, deltaT);
        BIGVBEKFSEQ_X_d = Ft(BIGVBEKFSEQ_X_d, deltaT);
        BIGVBEKFSEQ_Px_d = F*BIGVBEKFSEQ_Px_d*F'+ Q;
        BIGVBEKFSEQ_IGa1_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGa1_d; BIGVBEKFSEQ_IGa2_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGa2_d; % heuristic dynamic
        BIGVBEKFSEQ_IGb1_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGb1_d; BIGVBEKFSEQ_IGb2_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGb2_d; % heuristic dynamic
        BIGVBEKFSEQ_IGa3_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGa3_d; BIGVBEKFSEQ_IGa4_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGa4_d; % heuristic dynamic
        BIGVBEKFSEQ_IGb3_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGb3_d; BIGVBEKFSEQ_IGb4_d = BIGVBEKFSEQ_pho_d * BIGVBEKFSEQ_IGb4_d; % heuristic dynamic
        % measurement update
        H = JabH(BIGVBEKFSEQ_X_d, SensorsPoint); 
        % Sensors 1
        BIGVBEKFSEQ_IGa1_d = 0.5 + BIGVBEKFSEQ_IGa1_d; % VB step1 
        tmpBIGVBEKFSEQ_BerZ1t_d = BIGVBEKFSEQ_BerZt_d; 
        BIGVBEKFSEQ_BetaE1t_d = BIGVBEKFSEQ_BetaE0_d;  
        BIGVBEKFSEQ_BetaF1t_d = BIGVBEKFSEQ_BetaF0_d; 
        tmpBIGVBEKFSEQ_X_d = BIGVBEKFSEQ_X_d; 
        tmpBIGVBEKFSEQ_Px_d = BIGVBEKFSEQ_Px_d; 
        tmpBIGVBEKFSEQ_IGb1_d = BIGVBEKFSEQ_IGb1_d; 
        % Sensors 1
        BIGVBEKFSEQ_Hk1_d = H(1,:);
        for i = 1 : BIGVBEKFSEQ_it_max_d % VB iterate dynamic times
            if tmpBIGVBEKFSEQ_BerZ1t_d > BIGVBEKFSEQ_BerZt_TH 
                BIGVBEKFSEQ_X_d_pre = BIGVBEKFSEQ_X_d; 
                BIGVBEKFSEQ_R1_d = BIGVBEKFSEQ_IGb1_d / BIGVBEKFSEQ_IGa1_d; % VB-E step begin
                BIGVBEKFSEQ_Sk_d = BIGVBEKFSEQ_Hk1_d * tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk1_d'+ BIGVBEKFSEQ_R1_d / tmpBIGVBEKFSEQ_BerZ1t_d; 
                BIGVBEKFSEQ_Kk_d = tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk1_d' / BIGVBEKFSEQ_Sk_d; 
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d + BIGVBEKFSEQ_Kk_d * (Ypse(1,k) - Ht(tmpBIGVBEKFSEQ_X_d, SensorsPoint(1,:))); 
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d - BIGVBEKFSEQ_Kk_d * BIGVBEKFSEQ_Sk_d * BIGVBEKFSEQ_Kk_d'; % VB-E step done 
                tmpInnovation = (Ypse(1,k) - Ht(BIGVBEKFSEQ_X_d, SensorsPoint(1,:)))^2;
                BIGVBEKFSEQ_Bt = tmpInnovation + BIGVBEKFSEQ_Hk1_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk1_d'; 
                tmpP0 =psi(BIGVBEKFSEQ_BetaE1t_d + BIGVBEKFSEQ_BetaF1t_d ); 
                tmpP1 = exp(psi(BIGVBEKFSEQ_BetaE1t_d)- tmpP0 - 0.5 * trace(BIGVBEKFSEQ_Bt / BIGVBEKFSEQ_R1_d));
                tmpBIGVBEKFSEQ_BerZ1t_d = tmpP1 / (tmpP1 + exp(psi(BIGVBEKFSEQ_BetaF1t_d ) - tmpP0) );  
                BIGVBEKFSEQ_IGb1_d = tmpBIGVBEKFSEQ_IGb1_d + ... % VB-M step begin 
                    0.5 * tmpBIGVBEKFSEQ_BerZ1t_d * tmpInnovation + ... 
                    0.5 * tmpBIGVBEKFSEQ_BerZ1t_d * (BIGVBEKFSEQ_Hk1_d*BIGVBEKFSEQ_Px_d*BIGVBEKFSEQ_Hk1_d'); 
                BIGVBEKFSEQ_BetaE1t_d = BIGVBEKFSEQ_BetaE0_d + tmpBIGVBEKFSEQ_BerZ1t_d; 
                BIGVBEKFSEQ_BetaF1t_d = BIGVBEKFSEQ_BetaF0_d + 1 - tmpBIGVBEKFSEQ_BerZ1t_d; % VB-M step done 
                if(norm(BIGVBEKFSEQ_X_d - BIGVBEKFSEQ_X_d_pre) < BIGVBEKFSEQ_E_d) 
                    break;
                end
            else
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d;
                BIGVBEKFSEQ_IGb1_d = tmpBIGVBEKFSEQ_IGb1_d; 
                BIGVBEKFSEQ_IGa1_d = BIGVBEKFSEQ_IGa1_d - 0.5;
                BIGVBEKFSEQ_R1_d = BIGVBEKFSEQ_IGb1_d / BIGVBEKFSEQ_IGa1_d; 
                break;
            end
        end
        resBIGVBKFSEQ_1st_it_d(:,k) = i;
        % Sensors 2
        BIGVBEKFSEQ_IGa2_d = 0.5 + BIGVBEKFSEQ_IGa2_d; % VB step1
        tmpBIGVBEKFSEQ_BerZ2t_d = BIGVBEKFSEQ_BerZt_d; 
        BIGVBEKFSEQ_BetaE2t_d = BIGVBEKFSEQ_BetaE0_d; 
        BIGVBEKFSEQ_BetaF2t_d = BIGVBEKFSEQ_BetaF0_d;
        tmpBIGVBEKFSEQ_X_d = BIGVBEKFSEQ_X_d; 
        tmpBIGVBEKFSEQ_Px_d = BIGVBEKFSEQ_Px_d; 
        tmpBIGVBEKFSEQ_IGb2_d = BIGVBEKFSEQ_IGb2_d; 
        BIGVBEKFSEQ_Hk2_d = H(2,:);
        for i = 1 : BIGVBEKFSEQ_it_max_d % VB iterate dynamic times           
            if tmpBIGVBEKFSEQ_BerZ2t_d > BIGVBEKFSEQ_BerZt_TH
                BIGVBEKFSEQ_X_d_pre = BIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_R2_d = BIGVBEKFSEQ_IGb2_d / BIGVBEKFSEQ_IGa2_d; % VB-E step begin
                BIGVBEKFSEQ_Sk_d = BIGVBEKFSEQ_Hk2_d * tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk2_d'+ BIGVBEKFSEQ_R2_d / tmpBIGVBEKFSEQ_BerZ2t_d;
                BIGVBEKFSEQ_Kk_d = tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk2_d' / BIGVBEKFSEQ_Sk_d;
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d + BIGVBEKFSEQ_Kk_d * (Ypse(2,k) - Ht(tmpBIGVBEKFSEQ_X_d, SensorsPoint(2,:)));
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d - BIGVBEKFSEQ_Kk_d * BIGVBEKFSEQ_Sk_d * BIGVBEKFSEQ_Kk_d'; % VB-E step done
                tmpInnovation = (Ypse(2,k) - Ht(BIGVBEKFSEQ_X_d, SensorsPoint(2,:)))^2;
                BIGVBEKFSEQ_Bt = tmpInnovation + BIGVBEKFSEQ_Hk2_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk2_d';
                tmpP0 =psi(BIGVBEKFSEQ_BetaE2t_d + BIGVBEKFSEQ_BetaF2t_d ); 
                tmpP1 = exp(psi(BIGVBEKFSEQ_BetaE2t_d)- tmpP0 - 0.5 * trace(BIGVBEKFSEQ_Bt / BIGVBEKFSEQ_R2_d));
                tmpBIGVBEKFSEQ_BerZ2t_d = tmpP1 / (tmpP1 + exp(psi(BIGVBEKFSEQ_BetaF2t_d ) - tmpP0) );  
                BIGVBEKFSEQ_IGb2_d = tmpBIGVBEKFSEQ_IGb2_d + ... % VB-M step begin 
                    0.5 * tmpBIGVBEKFSEQ_BerZ2t_d * tmpInnovation + ... 
                    0.5 * tmpBIGVBEKFSEQ_BerZ2t_d * (BIGVBEKFSEQ_Hk2_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk2_d'); 
                BIGVBEKFSEQ_BetaE2t_d = BIGVBEKFSEQ_BetaE0_d + tmpBIGVBEKFSEQ_BerZ2t_d; 
                BIGVBEKFSEQ_BetaF2t_d = BIGVBEKFSEQ_BetaF0_d + 1 - tmpBIGVBEKFSEQ_BerZ2t_d; % VB-M step done
                if(norm(BIGVBEKFSEQ_X_d - BIGVBEKFSEQ_X_d_pre) < BIGVBEKFSEQ_E_d)
                    break;
                end
            else
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d; 
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d; 
                BIGVBEKFSEQ_IGb2_d = tmpBIGVBEKFSEQ_IGb2_d; 
                BIGVBEKFSEQ_IGa2_d = BIGVBEKFSEQ_IGa2_d - 0.5; 
                BIGVBEKFSEQ_R2_d = BIGVBEKFSEQ_IGb2_d / BIGVBEKFSEQ_IGa2_d; 
                break;
            end
        end
        resBIGVBKFSEQ_2nd_it_d(:,k) = i;
        % Sensors 3
        BIGVBEKFSEQ_IGa3_d = 0.5 + BIGVBEKFSEQ_IGa3_d; % VB step1
        tmpBIGVBEKFSEQ_BerZ3t_d = BIGVBEKFSEQ_BerZt_d;
        BIGVBEKFSEQ_BetaE3t_d = BIGVBEKFSEQ_BetaE0_d; 
        BIGVBEKFSEQ_BetaF3t_d = BIGVBEKFSEQ_BetaF0_d; 
        tmpBIGVBEKFSEQ_X_d = BIGVBEKFSEQ_X_d; 
        tmpBIGVBEKFSEQ_Px_d = BIGVBEKFSEQ_Px_d; 
        tmpBIGVBEKFSEQ_IGb3_d = BIGVBEKFSEQ_IGb3_d; 
        BIGVBEKFSEQ_Hk3_d = H(3,:);
        for i = 1 : BIGVBEKFSEQ_it_max_d % VB iterate dynamic times
            if tmpBIGVBEKFSEQ_BerZ3t_d > BIGVBEKFSEQ_BerZt_TH
                BIGVBEKFSEQ_X_d_pre = BIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_R3_d =  BIGVBEKFSEQ_IGb3_d / BIGVBEKFSEQ_IGa3_d; % VB-E step begin
                BIGVBEKFSEQ_Sk_d = BIGVBEKFSEQ_Hk3_d * tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk3_d'+ BIGVBEKFSEQ_R3_d / tmpBIGVBEKFSEQ_BerZ3t_d;
                BIGVBEKFSEQ_Kk_d = tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk3_d' / BIGVBEKFSEQ_Sk_d;
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d + BIGVBEKFSEQ_Kk_d * (Ypse(3,k) - Ht(tmpBIGVBEKFSEQ_X_d, SensorsPoint(3,:)));
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d - BIGVBEKFSEQ_Kk_d * BIGVBEKFSEQ_Sk_d * BIGVBEKFSEQ_Kk_d'; % VB-E step done
                tmpInnovation = (Ypse(3,k) - Ht(BIGVBEKFSEQ_X_d, SensorsPoint(3,:)))^2;
                BIGVBEKFSEQ_Bt = tmpInnovation + BIGVBEKFSEQ_Hk3_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk3_d';
                tmpP0 =psi(BIGVBEKFSEQ_BetaE3t_d + BIGVBEKFSEQ_BetaF3t_d ); 
                tmpP1 = exp(psi(BIGVBEKFSEQ_BetaE3t_d)- tmpP0 - 0.5 * trace(BIGVBEKFSEQ_Bt / BIGVBEKFSEQ_R3_d));
                tmpBIGVBEKFSEQ_BerZ3t_d = tmpP1 / (tmpP1 + exp(psi(BIGVBEKFSEQ_BetaF3t_d ) - tmpP0) );  
                BIGVBEKFSEQ_IGb3_d =  tmpBIGVBEKFSEQ_IGb3_d + ... % VB-M step begin
                    0.5 * tmpBIGVBEKFSEQ_BerZ3t_d * tmpInnovation + ...
                    0.5 * tmpBIGVBEKFSEQ_BerZ3t_d * (BIGVBEKFSEQ_Hk3_d*BIGVBEKFSEQ_Px_d*BIGVBEKFSEQ_Hk3_d');
                BIGVBEKFSEQ_BetaE3t_d = BIGVBEKFSEQ_BetaE0_d + tmpBIGVBEKFSEQ_BerZ3t_d;
                BIGVBEKFSEQ_BetaF3t_d = BIGVBEKFSEQ_BetaF0_d + 1 - tmpBIGVBEKFSEQ_BerZ3t_d; % VB-M step done
                if(norm(BIGVBEKFSEQ_X_d - BIGVBEKFSEQ_X_d_pre) < BIGVBEKFSEQ_E_d)
                    break;
                end
            else
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d;
                BIGVBEKFSEQ_IGb3_d = tmpBIGVBEKFSEQ_IGb3_d; 
                BIGVBEKFSEQ_IGa3_d = BIGVBEKFSEQ_IGa3_d - 0.5;
                BIGVBEKFSEQ_R3_d = BIGVBEKFSEQ_IGb3_d/BIGVBEKFSEQ_IGa3_d;
                break;
            end
        end
        resBIGVBKFSEQ_3rd_it_d(:,k) = i;
        % Sensors 4
        BIGVBEKFSEQ_IGa4_d = 0.5 + BIGVBEKFSEQ_IGa4_d; % VB step1
        tmpBIGVBEKFSEQ_BerZ4t_d = BIGVBEKFSEQ_BerZt_d;
        BIGVBEKFSEQ_BetaE4t_d = BIGVBEKFSEQ_BetaE0_d; 
        BIGVBEKFSEQ_BetaF4t_d = BIGVBEKFSEQ_BetaF0_d;
        tmpBIGVBEKFSEQ_X_d = BIGVBEKFSEQ_X_d; 
        tmpBIGVBEKFSEQ_Px_d = BIGVBEKFSEQ_Px_d; 
        tmpBIGVBEKFSEQ_IGb4_d = BIGVBEKFSEQ_IGb4_d;
        BIGVBEKFSEQ_Hk4_d = H(4,:);
        for i = 1 : BIGVBEKFSEQ_it_max_d % VB iterate dynamic times
            if tmpBIGVBEKFSEQ_BerZ4t_d > BIGVBEKFSEQ_BerZt_TH
                BIGVBEKFSEQ_X_d_pre = BIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_R4_d =  BIGVBEKFSEQ_IGb4_d / BIGVBEKFSEQ_IGa4_d; % VB-E step begin
                BIGVBEKFSEQ_Sk_d = BIGVBEKFSEQ_Hk4_d * tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk4_d'+ BIGVBEKFSEQ_R4_d / tmpBIGVBEKFSEQ_BerZ4t_d;
                BIGVBEKFSEQ_Kk_d = tmpBIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk4_d' / BIGVBEKFSEQ_Sk_d;
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d + BIGVBEKFSEQ_Kk_d * (Ypse(4,k) - Ht(tmpBIGVBEKFSEQ_X_d, SensorsPoint(4,:)));
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d - BIGVBEKFSEQ_Kk_d * BIGVBEKFSEQ_Sk_d * BIGVBEKFSEQ_Kk_d'; % VB-E step done
                tmpInnovation = (Ypse(4,k) - Ht(BIGVBEKFSEQ_X_d, SensorsPoint(4,:)))^2;
                BIGVBEKFSEQ_Bt = tmpInnovation + BIGVBEKFSEQ_Hk4_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk4_d';
                tmpP0 =psi(BIGVBEKFSEQ_BetaE4t_d + BIGVBEKFSEQ_BetaF4t_d ); 
                tmpP1 = exp(psi(BIGVBEKFSEQ_BetaE4t_d)- tmpP0 - 0.5 * trace(BIGVBEKFSEQ_Bt / BIGVBEKFSEQ_R4_d));
                tmpBIGVBEKFSEQ_BerZ4t_d = tmpP1 / (tmpP1 + exp(psi(BIGVBEKFSEQ_BetaF4t_d ) - tmpP0) );  
                BIGVBEKFSEQ_IGb4_d = tmpBIGVBEKFSEQ_IGb4_d + ... % VB-M step begin
                    0.5 * tmpBIGVBEKFSEQ_BerZ4t_d * tmpInnovation + ...
                    0.5 * tmpBIGVBEKFSEQ_BerZ4t_d * (BIGVBEKFSEQ_Hk4_d * BIGVBEKFSEQ_Px_d * BIGVBEKFSEQ_Hk4_d');
                BIGVBEKFSEQ_BetaE4t_d = BIGVBEKFSEQ_BetaE0_d + tmpBIGVBEKFSEQ_BerZ4t_d;
                BIGVBEKFSEQ_BetaF4t_d = BIGVBEKFSEQ_BetaF0_d + 1 - tmpBIGVBEKFSEQ_BerZ4t_d; % VB-M step done
                if(norm(BIGVBEKFSEQ_X_d - BIGVBEKFSEQ_X_d_pre) < BIGVBEKFSEQ_E_d)
                    break;
                end
            else
                BIGVBEKFSEQ_X_d = tmpBIGVBEKFSEQ_X_d;
                BIGVBEKFSEQ_Px_d = tmpBIGVBEKFSEQ_Px_d;
                BIGVBEKFSEQ_IGb4_d = tmpBIGVBEKFSEQ_IGb4_d; 
                BIGVBEKFSEQ_IGa4_d = BIGVBEKFSEQ_IGa4_d - 0.5;
                BIGVBEKFSEQ_R4_d = BIGVBEKFSEQ_IGb4_d / BIGVBEKFSEQ_IGa4_d;
                break;
            end
        end
        tBIGVBKFSEQEnd = toc;
        resBIGVBKFSEQ_4th_it_d(:,k) = i;
        %res
        resBIGVBKFSEQ_it_d = resBIGVBKFSEQ_1st_it_d + resBIGVBKFSEQ_2nd_it_d + ...
            resBIGVBKFSEQ_3rd_it_d + resBIGVBKFSEQ_4th_it_d;
        resBIGVBEKFSEQ_X_d(:,k) = BIGVBEKFSEQ_X_d;
        resBIGVBEKFSEQ_R_d(:,k) = [BIGVBEKFSEQ_R1_d; BIGVBEKFSEQ_R2_d;...
            BIGVBEKFSEQ_R3_d; BIGVBEKFSEQ_R4_d];
        resBIGVBSEQZt(:,k) = [tmpBIGVBEKFSEQ_BerZ1t_d; tmpBIGVBEKFSEQ_BerZ2t_d;...
            tmpBIGVBEKFSEQ_BerZ3t_d; tmpBIGVBEKFSEQ_BerZ4t_d];
        resBIGVBEKFSEQ_RMSE_d(:,k) = sqrt(trace(BIGVBEKFSEQ_Px_d));
        resBIGVBKFSEQTime(:,k) = tBIGVBKFSEQEnd;
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
    RMSE_EKF3p86_X(mc, 1) = rmse(resEKF3p86_X(1, :), Xpse(1, 2 : end));
    RMSE_EKF3p86_X(mc, 2) = rmse(resEKF3p86_X(2, :), Xpse(2, 2 : end));
    RMSE_EKF3p86_X(mc, 3) = rmse(resEKF3p86_X(3, :), Xpse(3, 2 : end));
    RMSE_EKF3p86_X(mc, 4) = rmse(resEKF3p86_X(4, :), Xpse(4, 2 : end));
    RMSE_EKF3p86_X(mc, 5) = rmse(resEKF3p86_X(5, :), Xpse(5, 2 : end));
    % MC EKF result
    RMSE_EKF100_X(mc, 1) = rmse(resEKF100_X(1, :), Xpse(1, 2 : end));
    RMSE_EKF100_X(mc, 2) = rmse(resEKF100_X(2, :), Xpse(2, 2 : end));
    RMSE_EKF100_X(mc, 3) = rmse(resEKF100_X(3, :), Xpse(3, 2 : end));
    RMSE_EKF100_X(mc, 4) = rmse(resEKF100_X(4, :), Xpse(4, 2 : end));
    RMSE_EKF100_X(mc, 5) = rmse(resEKF100_X(5, :), Xpse(5, 2 : end));
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
    RMSE_BIGVBEKF_X(mc, 1) = rmse(resBIGVBEKF_X_d(1, :), Xpse(1, 2 : end));
    RMSE_BIGVBEKF_X(mc, 2) = rmse(resBIGVBEKF_X_d(2, :), Xpse(2, 2 : end));
    RMSE_BIGVBEKF_X(mc, 3) = rmse(resBIGVBEKF_X_d(3, :), Xpse(3, 2 : end));
    RMSE_BIGVBEKF_X(mc, 4) = rmse(resBIGVBEKF_X_d(4, :), Xpse(4, 2 : end));
    RMSE_BIGVBEKF_X(mc, 5) = rmse(resBIGVBEKF_X_d(5, :), Xpse(5, 2 : end));
    RMSE_BIGVBEKF_R(mc, 1) = rmse(resBIGVBEKF_R_d(1, :), R(1, :));
    RMSE_BIGVBEKF_R(mc, 2) = rmse(resBIGVBEKF_R_d(2, :), R(2, :));
    RMSE_BIGVBEKF_R(mc, 3) = rmse(resBIGVBEKF_R_d(3, :), R(3, :));
    RMSE_BIGVBEKF_R(mc, 4) = rmse(resBIGVBEKF_R_d(4, :), R(4, :));
    % MC BIWVBEKF result
    RMSE_BIGVBEKFSEQ_X(mc, 1) = rmse(resBIGVBEKFSEQ_X_d(1, :), Xpse(1, 2 : end));
    RMSE_BIGVBEKFSEQ_X(mc, 2) = rmse(resBIGVBEKFSEQ_X_d(2, :), Xpse(2, 2 : end));
    RMSE_BIGVBEKFSEQ_X(mc, 3) = rmse(resBIGVBEKFSEQ_X_d(3, :), Xpse(3, 2 : end));
    RMSE_BIGVBEKFSEQ_X(mc, 4) = rmse(resBIGVBEKFSEQ_X_d(4, :), Xpse(4, 2 : end));
    RMSE_BIGVBEKFSEQ_X(mc, 5) = rmse(resBIGVBEKFSEQ_X_d(5, :), Xpse(5, 2 : end));
    RMSE_BIGVBEKFSEQ_R(mc, 1) = rmse(resBIGVBEKFSEQ_R_d(1, :), R(1, :));
    RMSE_BIGVBEKFSEQ_R(mc, 2) = rmse(resBIGVBEKFSEQ_R_d(2, :), R(2, :));
    RMSE_BIGVBEKFSEQ_R(mc, 3) = rmse(resBIGVBEKFSEQ_R_d(3, :), R(3, :));
    RMSE_BIGVBEKFSEQ_R(mc, 4) = rmse(resBIGVBEKFSEQ_R_d(4, :), R(4, :));
    % MC new_aprivbkf result
    RMSE_NEWAPRIVBKF_X(mc, 1) = rmse(resNew_aprivbkf_comp_X(1,:), Xpse(1, 2 : end));
    RMSE_NEWAPRIVBKF_X(mc, 2) = rmse(resNew_aprivbkf_comp_X(2,:), Xpse(2, 2 : end));
    RMSE_NEWAPRIVBKF_X(mc, 3) = rmse(resNew_aprivbkf_comp_X(3,:), Xpse(3, 2 : end));
    RMSE_NEWAPRIVBKF_X(mc, 4) = rmse(resNew_aprivbkf_comp_X(4,:), Xpse(4, 2 : end));
    RMSE_NEWAPRIVBKF_X(mc, 5) = rmse(resNew_aprivbkf_comp_X(5,:), Xpse(5, 2 : end));
    RMSE_NEWAPRIVBKF_R(mc, 1) = rmse(resNew_aprivbkf_comp_R(1,:), R(1, :));
    RMSE_NEWAPRIVBKF_R(mc, 2) = rmse(resNew_aprivbkf_comp_R(2,:), R(2, :));
    RMSE_NEWAPRIVBKF_R(mc, 3) = rmse(resNew_aprivbkf_comp_R(3,:), R(3, :));
    RMSE_NEWAPRIVBKF_R(mc, 4) = rmse(resNew_aprivbkf_comp_R(4,:), R(4, :));
    % MC swrkf result
    RMSE_SWRKF_X(mc, 1) = rmse(resSwrkf_X(1,:), Xpse(1, 2 : end));
    RMSE_SWRKF_X(mc, 2) = rmse(resSwrkf_X(2,:), Xpse(2, 2 : end));
    RMSE_SWRKF_X(mc, 3) = rmse(resSwrkf_X(3,:), Xpse(3, 2 : end));
    RMSE_SWRKF_X(mc, 4) = rmse(resSwrkf_X(4,:), Xpse(4, 2 : end));
    RMSE_SWRKF_X(mc, 5) = rmse(resSwrkf_X(5,:), Xpse(5, 2 : end));
    RMSE_SWRKF_R(mc, 1) = rmse(resSwrkf_R(1,:), R(1, :));
    RMSE_SWRKF_R(mc, 2) = rmse(resSwrkf_R(2,:), R(2, :));
    RMSE_SWRKF_R(mc, 3) = rmse(resSwrkf_R(3,:), R(3, :));
    RMSE_SWRKF_R(mc, 4) = rmse(resSwrkf_R(4,:), R(4, :));
    % MC ssmkf result
    RMSE_SSMKF_X(mc, 1) = rmse(resSsmkf_X(1,:), Xpse(1, 2 : end));
    RMSE_SSMKF_X(mc, 2) = rmse(resSsmkf_X(2,:), Xpse(2, 2 : end));
    RMSE_SSMKF_X(mc, 3) = rmse(resSsmkf_X(3,:), Xpse(3, 2 : end));
    RMSE_SSMKF_X(mc, 4) = rmse(resSsmkf_X(4,:), Xpse(4, 2 : end));
    RMSE_SSMKF_X(mc, 5) = rmse(resSsmkf_X(5,:), Xpse(5, 2 : end));
    RMSE_SSMKF_R(mc, 1) = rmse(resSsmkf_R(1,:), R(1, :));
    RMSE_SSMKF_R(mc, 2) = rmse(resSsmkf_R(2,:), R(2, :));
    RMSE_SSMKF_R(mc, 3) = rmse(resSsmkf_R(3,:), R(3, :));
    RMSE_SSMKF_R(mc, 4) = rmse(resSsmkf_R(4,:), R(4, :));
    % MC RSTKF result
    RMSE_RSTKF_X(mc, 1) = rmse(resRSTEKF_X(1,:), Xpse(1, 2 : end));
    RMSE_RSTKF_X(mc, 2) = rmse(resRSTEKF_X(2,:), Xpse(2, 2 : end));
    RMSE_RSTKF_X(mc, 3) = rmse(resRSTEKF_X(3,:), Xpse(3, 2 : end));
    RMSE_RSTKF_X(mc, 4) = rmse(resRSTEKF_X(4,:), Xpse(4, 2 : end));
    RMSE_RSTKF_X(mc, 5) = rmse(resRSTEKF_X(5,:), Xpse(5, 2 : end));
    RMSE_RSTKF_R(mc, 1) = rmse(resRSTEKF_R(1,:), R(1, :));
    RMSE_RSTKF_R(mc, 2) = rmse(resRSTEKF_R(2,:), R(2, :));
    RMSE_RSTKF_R(mc, 3) = rmse(resRSTEKF_R(3,:), R(3, :));
    RMSE_RSTKF_R(mc, 4) = rmse(resRSTEKF_R(4,:), R(4, :));
    % MC mckf result
    RMSE_MCKF_X(mc, 1) = rmse(resMCEKF_X(1,:), Xpse(1, 2 : end));
    RMSE_MCKF_X(mc, 2) = rmse(resMCEKF_X(2,:), Xpse(2, 2 : end));
    RMSE_MCKF_X(mc, 3) = rmse(resMCEKF_X(3,:), Xpse(3, 2 : end));
    RMSE_MCKF_X(mc, 4) = rmse(resMCEKF_X(4,:), Xpse(4, 2 : end));
    RMSE_MCKF_X(mc, 5) = rmse(resMCEKF_X(5,:), Xpse(5, 2 : end));
    RMSE_MCKF_R(mc, 1) = rmse(resMCEKF_R(1,:), R(1, :));
    RMSE_MCKF_R(mc, 2) = rmse(resMCEKF_R(2,:), R(2, :));
    RMSE_MCKF_R(mc, 3) = rmse(resMCEKF_R(3,:), R(3, :));
    RMSE_MCKF_R(mc, 4) = rmse(resMCEKF_R(4,:), R(4, :));
    % MC Time result
    MCCHIKFSEQ3p86Time(mc) = mean(resSEQKFCHI3p86Time);
    MCCHIKFSEQ100Time(mc) = mean(resSEQKFCHI100Time);
    MCVBKFTime(mc) = mean(resVBKFTime);
    MCRSTKFTime(mc) = mean(resRSTKFTime);
    MCNew_aprivbkfTime(mc) = mean(resNew_aprivbkfTime);
    MCSwrkfTime(mc) = mean(resSwrkfTime);
    MCSsmkfTime(mc) = mean(resSsmkfTime);
    MCMckfTime(mc) = mean(resMCKFTime);
    MCBIGVBKFTime(mc) = mean(resBIGVBKFTime);
    MCBIGVBKFSEQTime(mc) = mean(resBIGVBKFSEQTime);
end

%% plot
f1 = figure('Name','f1_estimated states');
f1.Units = "inches";
f1.Position = [5.833333333333333,2.979166666666667,11.6667,4.375];
subplot(4,9,[1:4,10:13,19:22,28:31]);
plot(Xpse(1,:), Xpse(3,:),'k-<'); hold on;
plot(Xpse(1,1), Xpse(3,1),'^'); hold on;
plot(Xpse(1,end), Xpse(3,end),'o'); hold on;
plot(Xpse(1,fix(outlierBegin/deltaT):fix(outlierEnd/deltaT)), Xpse(3,fix(outlierBegin/deltaT):fix(outlierEnd/deltaT)),'-'); hold on;
plot(Xpse(1,fix(susoutlierBegin/deltaT):fix(susoutlierEnd/deltaT)), Xpse(3,fix(susoutlierBegin/deltaT):fix(susoutlierEnd/deltaT)),'-'); hold on;
plot(resEKF3p86_X(1,:), resEKF3p86_X(3,:),'-'); hold on;
plot(resEKF100_X(1,:), resEKF100_X(3,:),'-'); hold on;
plot(resVBEKF_X_d(1,:), resVBEKF_X_d(3,:),'-'); hold on;
plot(resBIGVBEKF_X_d(1,:), resBIGVBEKF_X_d(3,:),'-'); hold on;
plot(resBIGVBEKFSEQ_X_d(1,:), resBIGVBEKFSEQ_X_d(3,:),'-'); hold on;
legend('True','start','end','Random Outlier Stage','Sustained Outlier Stage',...
    'chiKFSEQ-3.86','chiKFSEQ-100', 'VBEKF', 'BIWVBEKF', 'BIWVBEKFSEQ'); 
ax = gca; ax.Position = [0.081,0.11,0.332648401826484,0.815];
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
ax.XLim = [-300,100]; ax.YLim = [-300,200]; 
set(ylabel(['$P_E (m)$'],'Interpreter','latex'));
set(xlabel(['$P_N (m)$'],'Interpreter','latex'));
set(title(['Compared with traditional methods'],'Interpreter','latex'));
subplot(4,9,[5:8,14:17,23:26,32:35]);
plot(Xpse(1,:), Xpse(3,:),'k-<'); hold on;
plot(Xpse(1,1), Xpse(3,1),'^'); hold on;
plot(Xpse(1,end), Xpse(3,end),'o'); hold on;
plot(Xpse(1,fix(outlierBegin/deltaT):fix(outlierEnd/deltaT)), Xpse(3,fix(outlierBegin/deltaT):fix(outlierEnd/deltaT)),'-'); hold on;
plot(Xpse(1,fix(susoutlierBegin/deltaT):fix(susoutlierEnd/deltaT)), Xpse(3,fix(susoutlierBegin/deltaT):fix(susoutlierEnd/deltaT)),'-'); hold on;
plot(resNew_aprivbkf_comp_X(1,:), resNew_aprivbkf_comp_X(3,:),'-'); hold on;
plot(resSwrkf_X(1,:), resSwrkf_X(3,:),'-'); hold on;
plot(resSsmkf_X(1,:), resSsmkf_X(3,:),'-'); hold on;
plot(resRSTEKF_X(1,:), resRSTEKF_X(3,:),'-'); hold on;
plot(resMCEKF_X(1,:), resMCEKF_X(3,:),'-'); hold on;
plot(resBIGVBEKF_X_d(1,:), resBIGVBEKF_X_d(3,:),'-'); hold on;
plot(resBIGVBEKFSEQ_X_d(1,:), resBIGVBEKFSEQ_X_d(3,:),'-'); hold on;
legend('True','start','end', 'Random Outlier Stage','Sustained Outlier Stage', ...
    'new\_aprivbkf','swrkf', 'ssmkf',...
    'rstkf','mckf', 'BIWVBEKF', 'BIWVBEKFSEQ'); 
ax = gca; ax.Position = [0.4839,0.11,0.332648401826484,0.815];
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
ax.XLim = [-300,100]; ax.YLim = [-300,200];  
set(ylabel(['$P_E$'],'Interpreter','latex'));
set(xlabel(['$P_N$'],'Interpreter','latex'));
set(title(['Compared with cutting-edge methods'],'Interpreter','latex'));
subplot(4,9,9);  %S1
plot(Time,Ypse(1,:),'k'); hold on;ax = gca; ax.Position = [0.861,0.763367013932472,0.062246322285011,0.157741935483871];
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS1_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS1_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS1_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS1_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$y_{t,1} (m)$'],'Interpreter','latex'));
set(title(['Observers'],'Interpreter','latex'));
subplot(4,9,18); %S2
plot(Time,Ypse(2,:),'k'); hold on;ax = gca; ax.Position = [0.861,0.548172043010753,0.062246322196709,0.157741935483871];
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS2_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS2_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS2_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS2_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$y_{t,2} (m)$'],'Interpreter','latex'));
subplot(4,9,27); %S3
plot(Time,Ypse(3,:),'k'); hold on;
ax = gca;ax.Position = [0.861,0.329086021505376,0.062246322196709,0.157741935483871];
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';ax = gca;
picS3_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS3_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
picS3_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS3_Random,'edgealpha', 0.6, 'facealpha', 0.2);
set(ylabel(['$y_{t,3} (m)$'],'Interpreter','latex'));
subplot(4,9,36); %S4
plot(Time,Ypse(4,:),'k'); hold on;ax = gca; 
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);ax.Position = [0.861,0.11,0.062246322196709,0.157741935483871];
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$y_{t,4} (m)$'],'Interpreter','latex'));
set(xlabel(['$t (s)$'],'Interpreter','latex'));
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


f2 = figure('Name','f3_estimated R');
subplot(411); plot(Time,R(1,:)); hold on; plot(Time,resVBEKF_R_d(1,:)); 
plot(Time,resNew_aprivbkf_comp_R(1,:));  plot(Time,resSwrkf_R(1,:)); 
plot(Time,resSsmkf_R(1,:)); plot(Time,resRSTEKF_R(1,:)); 
plot(Time,resBIGVBEKF_R_d(1,:)); plot(Time,resBIGVBEKFSEQ_R_d(1,:)); hold on;
 ax = gca; ax.YScale='log';
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$\hat{R}_1$'],'Interpreter','latex'));
subplot(412); plot(Time,R(2,:)); hold on; plot(Time,resVBEKF_R_d(2,:)); 
plot(Time,resNew_aprivbkf_comp_R(2,:));  plot(Time,resSwrkf_R(2,:)); 
plot(Time,resSsmkf_R(2,:)); plot(Time,resRSTEKF_R(2,:)); 
plot(Time,resBIGVBEKF_R_d(2,:)); plot(Time,resBIGVBEKFSEQ_R_d(2,:)); 
 ax = gca; ax.YScale='log';
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$\hat{R}_2$'],'Interpreter','latex'));
subplot(413); plot(Time,R(3,:)); hold on; plot(Time,resVBEKF_R_d(3,:)); 
plot(Time,resNew_aprivbkf_comp_R(3,:));  plot(Time,resSwrkf_R(3,:)); 
plot(Time,resSsmkf_R(3,:)); plot(Time,resRSTEKF_R(3,:)); 
plot(Time,resBIGVBEKF_R_d(3,:)); plot(Time,resBIGVBEKFSEQ_R_d(3,:)); 
 ax = gca; ax.YScale='log';
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$\hat{R}_3$'],'Interpreter','latex'));
subplot(414); plot(Time,R(4,:)); hold on; plot(Time,resVBEKF_R_d(4,:)); 
plot(Time,resNew_aprivbkf_comp_R(4,:));  plot(Time,resSwrkf_R(4,:)); 
plot(Time,resSsmkf_R(4,:)); plot(Time,resRSTEKF_R(4,:)); 
plot(Time,resBIGVBEKF_R_d(4,:)); plot(Time,resBIGVBEKFSEQ_R_d(4,:)); 
 ax = gca; ax.YScale='log';
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
set(ylabel(['$\hat{R}_4$'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
legend('True', 'VBEKF', 'new\_aprivbekf','swrekf', 'ssmekf','rstekf',...
    'BIGVBEKF', 'BIGVBEKFSEQ');
sgtitle("Estimated Measurement Noise",'Fontsize',10,'Interpreter','Latex');
pos = get(f2,'Position');
set(f2,'Units','Inches');
set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

% plot CRLB and RMSE
f3 = figure('Name','f4_CRLB and RMSE'); 
plot(Time, [resEKF3p86_RMSE;resEKF100_RMSE;resVBEKF_RMSE_d;...
    resNew_aprivbkf_comp_RMSE;resSwrkf_RMSE;resSsmkf_RMSE;...
    resRSTEKF_RMSE;resMCEKF_RMSE;...
    resBIGVBEKF_RMSE_d;resBIGVBEKFSEQ_RMSE_d;...
    resCubCRLB_trLB]'); hold on; ax = gca; ax.YScale='log';
ax.YGrid = 'on';ax.YMinorGrid = 'on';ax.XGrid = 'on';ax.XMinorGrid = 'on';
picS4_Random = fill([outlierBegin, outlierEnd,outlierEnd,outlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'g','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Random,'edgealpha', 0.6, 'facealpha', 0.2);
picS4_Sustained = fill([susoutlierBegin, susoutlierEnd,susoutlierEnd,susoutlierBegin],...
    [ax.YLim(1),ax.YLim(1),ax.YLim(2),ax.YLim(2)],...
    'r','EdgeColor','r','LineStyle','--'); hold on;
set(picS4_Sustained,'edgealpha', 0.6, 'facealpha', 0.4);
legend('chiEKF-3.86','chiEKF-100', 'VBEKF', 'new\_aprivbekf','swrekf', ...
    'ssmekf','rstekf','mcekf', ...
    'BIWVBEKF', 'BIWVBEKFSEQ', 'CRLB'); 
set(ylabel(['RMSE error'],'Interpreter','latex')); 
set(xlabel(['Time($s$)'],'Interpreter','latex')); 
title("CRLB and RMSE",'Fontsize',10,'Interpreter','Latex'); 
pos = get(f3,'Position');
set(f3,'Units','Inches');
set(f3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

f4=figure('Name','f6_Bern Zt index');
subplot(4, 1, 1);
pcolor(Time, [1,2,3,4,5], [resBIGVBZt;zeros(size(resSEQEKFCHI3p86Index(1,:)))]);
set(ylabel('BIGVBEKF','Interpreter','latex'));
subplot(4, 1, 2);
pcolor(Time, [1,2,3,4,5], [resBIGVBSEQZt;zeros(size(resSEQEKFCHI3p86Index(1,:)))]);
set(ylabel('BIGVBEKFSEQ','Interpreter','latex'));
subplot(4, 1, 3);
pcolor(Time, [1,2,3,4,5], [resSEQEKFCHI3p86Index;zeros(size(resSEQEKFCHI3p86Index(1,:)))]);
set(ylabel('ChiEKF-3.86','Interpreter','latex'));
set(xlabel('Time($s$)','Interpreter','latex'));
sgtitle("Bernoulli Zt index  (yellow - accept, blue - reject)",'Fontsize',10,'Interpreter','Latex');
subplot(4, 1, 4);
pcolor(Time, [1,2,3,4,5], [resSEQEKFCHI100Index;zeros(size(resSEQEKFCHI100Index(1,:)))]);
set(ylabel('ChiEKF-100','Interpreter','latex'));
set(xlabel('Time($s$)','Interpreter','latex'));
sgtitle("Bernoulli Zt index  (yellow - accept, blue - reject)",'Fontsize',10,'Interpreter','Latex');
pos = get(f4,'Position');
set(f4,'Units','Inches');
set(f4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


% plot MC RMSE
if(MC_times > 1)

    f5=figure;
    subplot(2, 1, 1);
    semilogy(Time,[resSEQKFCHI3p86Time;resSEQKFCHI100Time;resVBKFTime;resNew_aprivbkfTime;...
        resSwrkfTime;resSsmkfTime;resRSTKFTime;resMCKFTime;resBIGVBKFTime;resBIGVBKFSEQTime]);
    legend('chiEKFSEQ-3.86','chiEKFSEQ-100','vbekf','new\_aprivbekf',...
        'swrekf','ssmekf','rstekf','mcekf',...
        'BIGVBEKF','BIGVBEKFSEQ');
    ax = gca; ax.XGrid = 'on';ax.XMinorGrid = 'on'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
    set(xlabel(['Time($s$)'],'Interpreter','latex'));
    set(ylabel('Elapsed Time($s$)','Interpreter','latex'));
    subplot(2, 1, 2);
    timeItems = categorical({'chiEKFSEQ-3.86','chiEKFSEQ-100','vbekf',...
        'new\_aprivbekf',...
        'swrekf','ssmekf','rstekf','mcekf','BIGVBEKF','BIGVBEKFSEQ'});
    timeItems = reordercats(timeItems,{'chiEKFSEQ-3.86','chiEKFSEQ-100','vbekf',...
        'new\_aprivbekf',...
        'swrekf','ssmekf','rstekf','mcekf','BIGVBEKF','BIGVBEKFSEQ'}); 
    meanTime = mean([MCCHIKFSEQ3p86Time,MCCHIKFSEQ100Time,MCVBKFTime,...
        MCNew_aprivbkfTime,MCSwrkfTime,MCSsmkfTime,MCRSTKFTime,...
        MCMckfTime,MCBIGVBKFTime,MCBIGVBKFSEQTime]);
    b=bar(timeItems,meanTime','FaceColor','flat');
    ax = gca; ax.XGrid = 'on';ax.XMinorGrid = 'on'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
    set(ylabel('Monte-Carlo Elapsed Time($s$)','Interpreter','latex'));
    xtips = b.XEndPoints;
    ytips = b.YEndPoints;
    labels = string(roundn(b.YData*1e6,-2));
    color =  lines(size(meanTime,2));
    for k = 1:size(meanTime,2)
        b.CData(k,:) = color(k,:);
        labels(k) = strcat(labels(k),'$\mu s$');
        text(xtips(k),ytips(k),labels(k),'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',8,'Color',color(k,:),'Interpreter','latex');
    end
    sgtitle("Time Consumption in Matlab of Simulation",'Fontsize',10,'Interpreter','Latex');
    pos = get(f5,'Position');
    set(f5,'Units','Inches');
    set(f5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

    C = lines(9);
    items={'chiKFSEQ-3.86','chiKFSEQ-100','VBEKF','new_aprivbekf',...
        'swrekf','ssmekf','rstekf','mcekf',...
        'BIGVBEKF','BIGVBEKFSEQ'};
    % state MC RMSE
    f6 = figure('Name','f5_state MC RMSE');
    subplot(231);boxplot([RMSE_EKF3p86_X(:,1),RMSE_EKF100_X(:,1),...
        RMSE_VBEKF_X(:,1), RMSE_NEWAPRIVBKF_X(:,1), ...
        RMSE_SWRKF_X(:,1),RMSE_SSMKF_X(:,1),RMSE_RSTKF_X(:,1),RMSE_MCKF_X(:,1), ...
        RMSE_BIGVBEKF_X(:,1),RMSE_BIGVBEKFSEQ_X(:,1)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Position $u$'],'Interpreter','latex'));
    set(ylabel(['$\delta p_E$'],'Interpreter','latex'));
    subplot(232);boxplot([RMSE_EKF3p86_X(:,2),RMSE_EKF100_X(:,2),...
        RMSE_VBEKF_X(:,2), RMSE_NEWAPRIVBKF_X(:,2), ...
        RMSE_SWRKF_X(:,2),RMSE_SSMKF_X(:,2),RMSE_RSTKF_X(:,2),RMSE_MCKF_X(:,2), ...
        RMSE_BIGVBEKF_X(:,2),RMSE_BIGVBEKFSEQ_X(:,2)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Position $v$'],'Interpreter','latex'));
    set(ylabel(['$\delta v_E$'],'Interpreter','latex'));
    subplot(233);boxplot([RMSE_EKF3p86_X(:,3),RMSE_EKF100_X(:,3), ...
        RMSE_VBEKF_X(:,3), RMSE_NEWAPRIVBKF_X(:,3), ...
        RMSE_SWRKF_X(:,3),RMSE_SSMKF_X(:,3),RMSE_RSTKF_X(:,3), RMSE_MCKF_X(:,3), ...
        RMSE_BIGVBEKF_X(:,3),RMSE_BIGVBEKFSEQ_X(:,3)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca;ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Velocity $u$'],'Interpreter','latex'));
    set(ylabel(['$\delta p_N$'],'Interpreter','latex'));
    subplot(234);boxplot([RMSE_EKF3p86_X(:,4),RMSE_EKF100_X(:,4),...
        RMSE_VBEKF_X(:,4), RMSE_NEWAPRIVBKF_X(:,4), ...
        RMSE_SWRKF_X(:,4),RMSE_SSMKF_X(:,4),RMSE_RSTKF_X(:,4),RMSE_MCKF_X(:,4), ...
        RMSE_BIGVBEKF_X(:,4),RMSE_BIGVBEKFSEQ_X(:,4)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Velocity $v$'],'Interpreter','latex'));
    set(ylabel(['$\delta v_N$'],'Interpreter','latex'));
    subplot(235);boxplot([RMSE_EKF3p86_X(:,5),RMSE_EKF100_X(:,5),...
        RMSE_VBEKF_X(:,5), RMSE_NEWAPRIVBKF_X(:,5), ...
        RMSE_SWRKF_X(:,5),RMSE_SSMKF_X(:,5),RMSE_RSTKF_X(:,5),RMSE_MCKF_X(:,5), ...
        RMSE_BIGVBEKF_X(:,5),RMSE_BIGVBEKFSEQ_X(:,5)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Velocity $v$'],'Interpreter','latex'));
    set(ylabel(['$\delta \omega$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo state RMSE Value",'Fontsize',10,'Interpreter','Latex');
       pos = get(f6,'Position');
    set(f6,'Units','Inches');
    set(f6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

    % measurement noise statistics MC RMSE
    items={'VBEKF','new_aprivbekf','swrekf','ssmekf','rstekf',...
        'BIGVBEKF','BIGVBEKFSEQ'};
    f7 = figure('Name','f6_measurement noise statistics MC RMSE');
    subplot(221);boxplot([RMSE_VBEKF_R(:,1),...
        RMSE_NEWAPRIVBKF_R(:,1), RMSE_SWRKF_R(:,1),RMSE_SSMKF_R(:,1),...
        RMSE_RSTKF_R(:,1), RMSE_BIGVBEKF_R(:,1), RMSE_BIGVBEKFSEQ_R(:,1)],...
        items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$\delta \hat{R}_{t1}$'],'Interpreter','latex'));
    ax = gca;ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    subplot(222);boxplot([RMSE_VBEKF_R(:,2), ...
        RMSE_NEWAPRIVBKF_R(:,2), RMSE_SWRKF_R(:,2),RMSE_SSMKF_R(:,2),...
        RMSE_RSTKF_R(:,2), RMSE_SWRKF_R(:,2), RMSE_BIGVBEKF_R(:,2)],...
        items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$\delta \hat{R}_{t2}$'],'Interpreter','latex'));
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    subplot(223);boxplot([RMSE_VBEKF_R(:,3), ...
        RMSE_NEWAPRIVBKF_R(:,3),  RMSE_SWRKF_R(:,3),RMSE_SSMKF_R(:,3),...
        RMSE_RSTKF_R(:,3),RMSE_SWRKF_R(:,3), RMSE_BIGVBEKF_R(:,3)],...
        items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$\delta \hat{R}_{t3}$'],'Interpreter','latex'));
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    subplot(224);boxplot([RMSE_VBEKF_R(:,4), ...
        RMSE_NEWAPRIVBKF_R(:,4), RMSE_SWRKF_R(:,4),RMSE_SSMKF_R(:,4),...
        RMSE_RSTKF_R(:,4), RMSE_SWRKF_R(:,4), RMSE_BIGVBEKF_R(:,4)],...
        items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$\delta \hat{R}_{t4}$'],'Interpreter','latex'));
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    sgtitle("Monte-Carlo measurement noise RMSE Value",'Fontsize',10,'Interpreter','Latex');
       pos = get(f7,'Position');
    set(f7,'Units','Inches');
    set(f7,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

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
