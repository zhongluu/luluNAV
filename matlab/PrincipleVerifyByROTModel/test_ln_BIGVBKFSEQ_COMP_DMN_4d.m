%% Inv-Gamma 4 dimension Dynamic Measurement Noise
% Yulu Zhong
% 4 state variables u, v, dot{u}, dot{v}
% linear observer
% U controler by acc. dot{dot{u}}, dot{dot{v}}
close all;
clear;
% true trajectory
deltaT = 0.1; % control value 0.1 sec.
% deltaT = 1;
stopT = 50; % 
X0 = [-20,8,0,0]'; % initial vel. 0 pos. (-20,8)
U = zeros(2, fix(stopT/deltaT)); % set acc.
% k=1;
% for t = 0:deltaT:stopT
%     if(t < 3)
%         U(1,k) = 3; % speed up from u dir.
%     elseif(t <4)
%          U(1,k) = 0;% uniform
%     elseif(t<5)
%         U(1,k) = -9;
%         U(2,k) = -8;% turn right
%     elseif(t<6)
%         U(2,k) = 4; 
%     elseif(t<7)
%         U(1,k) = 0;
%     end
%     k = k+1;
% end
outlierBegin = 10;
outlierEnd = 12;
outlierBegin2 = 20;
outlierEnd2 = 22;
outlierGm_Mu = zeros(2,2);

Xpse = zeros(4, fix(stopT/deltaT)); % prealloc true X
Ypse = zeros(2, fix(stopT/deltaT)); % prealloc true Y
Time = zeros(1, fix(stopT/deltaT));
Xtrue(:,1) = X0;
Xpse(:,1) = X0;
VB_D_Err_TH = 1e-4;
VB_it_Max = 100;
F = [eye(2),deltaT*eye(2);
    zeros(2,2),eye(2)];
B = [0.5*deltaT^2*eye(2);
    deltaT*eye(2)];
H = [eye(2),zeros(2,2)];
Q = [deltaT^3/3*eye(2),deltaT^2/2*eye(2);
     deltaT^2/2*eye(2), deltaT*eye(2)];

nominal_R =  0.64*eye(2);
abnomal_Lambda = 0.5;
abnomal_Alpha = 500;
sustainMu = 0;
R = 0.64*(2.* ones(2, fix(stopT/deltaT)) - cos(0.01*(0:1:fix(stopT/deltaT)-1)));

MC_times = 500; % Monte-Carlo times
RMSE_KF_X = zeros(MC_times,4); % prealloc MC result for chiKFSEQ
RMSE_VBKF_X_d = zeros(MC_times,4); % prealloc MC result for VBKF
RMSE_VBKF_R_d = zeros(MC_times,2); % prealloc MC result for VBKF
RMSE_RSTKF_X = zeros(MC_times,4);
RMSE_NEWAPRIVBKF_X = zeros(MC_times, 4); % prealloc MC result for resNew_aprivbkf
RMSE_NEWAPRIVBKF_R = zeros(MC_times,2); % prealloc MC result for resNew_aprivbkf
RMSE_SWRKF_X = zeros(MC_times, 4); % prealloc MC result for resSwrkf
RMSE_SWRKF_R = zeros(MC_times,2); % prealloc MC result for resSwrkf
RMSE_SSMKF_X = zeros(MC_times, 4); % prealloc MC result for resSsmkf
RMSE_MCKF_X = zeros(MC_times, 4); % prealloc MC result for resMCKF
RMSE_BIGVBKF_X = zeros(MC_times,4); % prealloc MC result for BIGVBEKF
RMSE_BIGVBKF_R = zeros(MC_times,2); % prealloc MC result for BIGVBEKF
RMSE_BIGVBKFSEQ_X = zeros(MC_times,4); % prealloc MC result for BIGVBEKFSEQ
RMSE_BIGVBKFSEQ_R = zeros(MC_times,2); % prealloc MC result for BIGVBEKFSEQ
MCCHIKFSEQTime = zeros(MC_times,1); % prealloc MC result for chi KF SEQ time
MCVBKFTime = zeros(MC_times,1); % prealloc MC result for VBKF
MCRSTKFTime = zeros(MC_times,1); % prealloc MC result for VBKFSEQ
MCNew_aprivbkfTime = zeros(MC_times,1); % prealloc MC result for resNew_aprivbkf
MCSwrkfTime = zeros(MC_times,1); % prealloc MC result for resSwrkf
MCSsmkfTime = zeros(MC_times,1); % prealloc MC result for resSsmkf
MCMckfTime = zeros(MC_times,1); % prealloc MC result for resMCKF
MCBIGVBKFTime = zeros(MC_times,1); % prealloc MC result for BIGVBEKF
MCBIGVBKFSEQTime = zeros(MC_times,1); % prealloc MC result for BIGVBEKFSEQ
for mc = 1:MC_times
    %% model stimulate
    k=1;
    for t = 0:deltaT:stopT-deltaT
        Xpse(:,k+1) = F*Xpse(:,k)+ B*U(:,k) + chol(Q)*randn(4,1);
        if (t > outlierBegin)&&(t <outlierEnd)
            outlierGm_Sigma = zeros(2,2,2);
            outlierGm_Sigma(:,:,1) = diag(R(:,k));
            abnomal_R = abnomal_Alpha * diag(R(:,k));
            outlierGm_Sigma(:,:,2) = abnomal_R;
            outlierGm_P = [1-abnomal_Lambda,abnomal_Lambda];
            outlierGm = gmdistribution(outlierGm_Mu, outlierGm_Sigma, outlierGm_P);
            Ypse(:,k) = H * Xpse(:,k+1) + random(outlierGm,1)';
        elseif (t > outlierBegin2) && (t <outlierEnd2)
%             Ypse(:,k) = H * Xpse(:,k+1) + mvnrnd(sustainMu * ones(2,1), diag(R(:,k)))';
            Ypse(:,k) = H * Xpse(:,k+1) + random(outlierGm,1)';
        else
            Ypse(:,k) = H * Xpse(:,k+1) + mvnrnd(zeros(2,1), diag(R(:,k)))';
        end
        Time(k) = t;
        k = k+1;
    end

    Xpse = Xpse(:,1:end-1);

    filter_P0 = 1*Q;
    filter_X0 = X0 + chol(filter_P0)*randn(4,1);
    
   
    %% KF
    resKF_X = zeros(4, fix(stopT/deltaT));
    resSEQKFCHI = zeros(2, fix(stopT/deltaT));
    resSEQKFCHIIndex = zeros(2, fix(stopT/deltaT));
    resKF_RMSE = zeros(1, fix(stopT/deltaT));
    resSEQKFCHITime = zeros(1, fix(stopT/deltaT));
    KF_Px = filter_P0;
    KF_X = filter_X0;
    KF_R = nominal_R;
    SEQEKFCHIth = 3.86;
    k=1;
    for t = 0 : deltaT : stopT-deltaT
%         tSEQEKFCHIStrat = tic;
        tic;
        % time update
        KF_X = F*KF_X+ B*U(:,k);
        KF_Px = F*KF_Px*F'+ Q;
        % measurement update
        KF_Hk = H;
        SEQEKFHn = size(KF_Hk,1);
        KF_R = diag(R(:,k));
        for i = 1:SEQEKFHn
            KF_Sk = KF_Hk(i,:)*KF_Px*KF_Hk(i,:)'+ KF_R(i,i);
            SEQEKFCHI = (Ypse(i,k) - KF_Hk(i,:)*KF_X)'/KF_Sk*(Ypse(i,k) - KF_Hk(i,:)*KF_X);
            resSEQKFCHI(i,k) = SEQEKFCHI;
            if (SEQEKFCHI < SEQEKFCHIth)
                resSEQKFCHIIndex(i,k) = 1;
                KF_Kk = KF_Px * KF_Hk(i,:)' / KF_Sk;
                KF_X = KF_X + KF_Kk * (Ypse(i,k) - KF_Hk(i,:) * KF_X);
                KF_Px = KF_Px - KF_Kk * KF_Sk * KF_Kk';
            end
        end
        tSEQEKFCHIEnd = toc;
        % res
        resKF_X(:,k) = KF_X;
        resKF_RMSE(:,k) = sqrt(trace(KF_Px));
        resSEQKFCHITime(:,k) = tSEQEKFCHIEnd;
        k=k+1;
    end
    %% VBKF  - InvGamma dynamic iteration
    resVBKF_X_d = zeros(4, fix(stopT/deltaT));
    resVBKF_R_d = zeros(2, fix(stopT/deltaT));
    resVBKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resVBKF_it_d = zeros(1, fix(stopT/deltaT));
    resVBKFTime = zeros(1, fix(stopT/deltaT));
    VBKF_X_d = filter_X0;
    VBKF_Px_d = filter_P0;  
    VBKF_IGa1_d = 1;VBKF_IGb1_d = 1; % Inv-Gamma param.
    VBKF_IGa2_d = 1;VBKF_IGb2_d = 1; % Inv-Gamma param.
    VBKF_it_max_d = 20; % iteration times
    VBKF_E_d = VB_D_Err_TH; % error threshold
    VBKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
%         tVBKFStrat = tic;
        tic;
        % time update
        VBKF_X_d = F*VBKF_X_d+ B*U(:,k);
        VBKF_Px_d = F*VBKF_Px_d*F'+ Q;
        VBKF_IGa1_d = VBKF_pho_d * VBKF_IGa1_d; 
        VBKF_IGa2_d = VBKF_pho_d * VBKF_IGa2_d; % heuristic dynamic
        VBKF_IGb1_d = VBKF_pho_d * VBKF_IGb1_d; 
        VBKF_IGb2_d = VBKF_pho_d * VBKF_IGb2_d; % heuristic dynamic
        % measurement update
        VBKF_Hk_d = H;
        VBKF_IGa1_d = 0.5 + VBKF_IGa1_d; % VB step1
        VBKF_IGa2_d = 0.5 + VBKF_IGa2_d; 
        tmpVBKF_X_d = VBKF_X_d; tmpVBKF_IGb1_d = VBKF_IGb1_d; 
        tmpVBKF_Px_d = VBKF_Px_d; tmpVBKF_IGb2_d = VBKF_IGb2_d;
        for i = 1 : VBKF_it_max_d % VB iterate dynamic times
            VBKF_X_d_pre = VBKF_X_d;
            VBKF_R_d = diag([VBKF_IGb1_d/VBKF_IGa1_d,VBKF_IGb2_d/VBKF_IGa2_d]); % VB-E step begin
            VBKF_Sk_d = VBKF_Hk_d*tmpVBKF_Px_d*VBKF_Hk_d'+ VBKF_R_d;
            VBKF_Kk_d = tmpVBKF_Px_d*VBKF_Hk_d'/VBKF_Sk_d;
            VBKF_X_d = tmpVBKF_X_d + VBKF_Kk_d*(Ypse(:,k) - VBKF_Hk_d*tmpVBKF_X_d);
            VBKF_Px_d = tmpVBKF_Px_d - VBKF_Kk_d*VBKF_Sk_d*VBKF_Kk_d'; % VB-E step done
            tmp = [tmpVBKF_IGb1_d; tmpVBKF_IGb2_d]+ ... % VB-M step begin
                0.5*(Ypse(:,k) - VBKF_Hk_d*VBKF_X_d).^2 + ...
                0.5*diag((VBKF_Hk_d*VBKF_Px_d*VBKF_Hk_d')); 
            VBKF_IGb1_d = tmp(1); VBKF_IGb2_d = tmp(2); % VB-M step done        
            if(norm(VBKF_X_d-VBKF_X_d_pre) < VBKF_E_d)
                break;
            end
        end
        tVBKFEnd = toc;
        resVBKF_it_d(:,k) = i;
        %res
        resVBKF_X_d(:,k) = VBKF_X_d;
        resVBKF_R_d(:,k) = diag(VBKF_R_d);
        resVBKF_RMSE_d(:,k) = sqrt(trace(VBKF_Px_d));
        resVBKFTime(:,k) = tVBKFEnd;
        k=k+1;
    end        
    %% RSTKF
    % res
    resRSTKF_X = zeros(4, fix(stopT / deltaT));
    resRSTKF_RMSE = zeros(1, fix(stopT / deltaT));
    resRSTKFTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    rstkf.xk= filter_X0;
    rstkf.Pxk = filter_P0;
    rstkf.Gammak = 1;   rstkf.Qk = Q; rstkf.Hk = H; rstkf.measIter = VBKF_it_max_d;
    rstkf.Phikk_1 = F; [rstkf.m, rstkf.n] = size(H);
    rstkf.Rk = nominal_R;
    k=1;
    for t = 0:deltaT:stopT-deltaT
%        tMckfStart = tic;
       tic;
       rstkf = akfupdate(rstkf, Ypse(:,k), 'B', 'RSTKF', 3, VB_D_Err_TH);
       tRstkfEnd = toc;
       % res
       resRSTKF_X(:,k) = rstkf.xk;
       resRSTKF_RMSE(:,k) = sqrt(trace(rstkf.Pxk));
       resRSTKFTime(:,k) = tRstkfEnd;
       k = k + 1;
    end
    %% A Slide Window Variational Adaptive Kalman Filter - new_aprivbkf
     % res 
     resNew_aprivbkf_X = zeros(4, fix(stopT / deltaT));
     resNew_aprivbkf_R = zeros(2, fix(stopT / deltaT));
     resNew_aprivbkf_RMSE = zeros(1, fix(stopT / deltaT));
     resNew_aprivbkfTime = zeros(1, fix(stopT/deltaT));
     % filter parameters set
     new_xapriv=filter_X0;
     new_Papriv=filter_P0;
     new_xapriv_A=[];
     new_Papriv_A=[];
     new_yapriv=0;
     new_Yapriv=zeros(4);
     new_uapriv=0;
     new_Uapriv=zeros(2);
     new_Qapriv=Q;
     new_Rapriv=nominal_R;
     new_Lapriv=5;
     rou=1-exp(-4);
     zA=[];
     k=1;
     for t = 0:deltaT:stopT-deltaT
%         tNew_aprivbkfStart = tic;
        tic;
        if t<=((new_Lapriv+1)*deltaT)
           zA=[zA Ypse(:,k)];
        else
           zA=[zA(:,2:end) Ypse(:,k)];
        end
        [new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,new_Qapriv,new_Rapriv,new_Ppapriv]=...
            new_aprivbkf(new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,F,H,zA,new_Qapriv,new_Rapriv,rou,new_Lapriv,k);
        tNew_aprivbkfEnd = toc;
        %res
        resNew_aprivbkf_X(:,k) = new_xapriv;
        resNew_aprivbkf_R(:,k) = diag(new_Rapriv);
        resNew_aprivbkf_RMSE(:,k) = sqrt(trace(new_Papriv));
        resNew_aprivbkfTime(:,k) = tNew_aprivbkfEnd;
        k=k+1;
     end
%% A Sliding Window Variational Outlier-Robust Kalman Filter Based on Student s t-Noise Modeling - swrkf
     % res       
     resSwrkf_X = zeros(4, fix(stopT / deltaT));
     resSwrkf_R = zeros(2, fix(stopT / deltaT));
     resSwrkf_RMSE = zeros(1, fix(stopT / deltaT));
     resSwrkfTime = zeros(1, fix(stopT / deltaT));
     % filter parameters set
     tao_R=5;
     tao_Q=5;
     epsilon=VB_D_Err_TH;
     rou=1-exp(-4);
     omega=5;
     nu=5;
     L=10*deltaT;
     zA=[];
     xsw=filter_X0;
     Psw=filter_P0;
     xsw_A=xsw;
     Psw_A=Psw;
     ysw_A=tao_Q;
     Ysw_A=tao_Q*Q;
     usw_A=tao_R;
     Usw_A=tao_R*nominal_R;
     Qsw=Q;
     Rsw=nominal_R;
     ksisw=[];
     lambdasw=[];
     k=1;
     for t = 0:deltaT:stopT-deltaT
%         tSwrkfStart = tic;
        tic;
        if t<=(L)
           zA=[zA Ypse(:,k)];
        else
           zA=[zA(:,2:end) Ypse(:,k)];
        end
        [xsw,Psw,xsw_A,Psw_A,xswnkNA,PswnkNA,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,ksisw,lambdasw,Nsw,M]=...
            swrkf(xsw_A,Psw_A,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,F,H,zA,ksisw,lambdasw,omega,nu,rou,L,k,VB_it_Max,epsilon);
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
    resSsmkf_X = zeros(4, fix(stopT / deltaT));
    resSsmkf_RMSE = zeros(1, fix(stopT / deltaT));
    resSsmkfTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    xssm_13=filter_X0;
    Pssm_13=filter_P0;
    N=VBKF_it_max_d;              %%%%%%最大变分迭代次数
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
%         tSsmkfStart = tic;
        tic;
        z = Ypse(:,k);
        [xssm_13,Pssm_13]=ssmkf(xssm_13,Pssm_13,F,H,z,Q0,R0,sigma,v_sq,N,epsilon,3);
        tSsmkfEnd = toc;
        % res
        resSsmkf_X(:,k) = xssm_13;
        resSsmkf_RMSE(:,k) = sqrt(trace(Pssm_13));
        resSsmkfTime(:,k) = tSsmkfEnd;
        k = k + 1;
    end
    %% maximum correntropy Kalman filter
    % res
    resMCKF_X = zeros(4, fix(stopT / deltaT));
    resMCKF_RMSE = zeros(1, fix(stopT / deltaT));
    resMCKFTime = zeros(1, fix(stopT / deltaT));
    % filter parameters set
    mckf.xk= filter_X0;
    mckf.Pxk = filter_P0;
    mckf.Gammak = 1;   mckf.Qk = Q; mckf.Hk = H; mckf.measIter = VBKF_it_max_d;
    mckf.Phikk_1 = F; [mckf.m, mckf.n] = size(H);
    mckf.Rk = nominal_R;
    k=1;
    for t = 0:deltaT:stopT-deltaT
%        tMckfStart = tic;
       tic;
       mckf = akfupdate(mckf, Ypse(:,k), 'B', 'MCKF',  5, VB_D_Err_TH);
       tMckfEnd = toc;
       % res
       resMCKF_X(:,k) = mckf.xk;
       resMCKF_RMSE(:,k) = sqrt(trace(mckf.Pxk));
       resMCKFTime(:,k) = tMckfEnd;
       k = k + 1;
    end
%% BIGVBKF (bern inv Gamma) - MINE
    resBIGVBKF_X_d = zeros(4, fix(stopT/deltaT));
    resBIGVBKF_R_d = zeros(2, fix(stopT/deltaT));
    resBIGVBKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resBIGVBKF_it_d = zeros(1, fix(stopT/deltaT));
    resBIGVBZt = zeros(2, fix(stopT/deltaT));
    resBIGVBKFTime = zeros(1, fix(stopT/deltaT));
    BIGVBKF_X_d = filter_X0;
    BIGVBKF_Px_d = filter_P0;  
    BIGVBKF_BerZt_d = 1; % Bernoulli param. 
    BIGVBKF_BerZt_TH = 0.1;
    BIGVBKF_BetaE0_d = 0.9; BIGVBKF_BetaF0_d = 0.1; % Beta params.
    BIGVBKF_IGa1_d = 1; BIGVBKF_IGb1_d = R(1,1);%BIWVBEKF_IGb1_d = 50^2; % Inv-Gamma param.
    BIGVBKF_IGa2_d = 1; BIGVBKF_IGb2_d = R(2,1);%BIWVBEKF_IGb2_d = 50^2; % Inv-Gamma param.
    BIGVBKF_it_max_d = VB_it_Max;
    BIGVBKF_E_d = VB_D_Err_TH; % error threshold
    BIGVBKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0 : deltaT : stopT - deltaT
%         tBIGVBKFStart = tic;
        tic;
        % time update
        BIGVBKF_X_d =  F*BIGVBKF_X_d+ B*U(:,k); 
        BIGVBKF_Px_d = F * BIGVBKF_Px_d * F' + Q; 
        BIGVBKF_IGa1_d = BIGVBKF_pho_d * BIGVBKF_IGa1_d; BIGVBKF_IGa2_d = BIGVBKF_pho_d * BIGVBKF_IGa2_d; % heuristic dynamic
        BIGVBKF_IGb1_d = BIGVBKF_pho_d * BIGVBKF_IGb1_d; BIGVBKF_IGb2_d = BIGVBKF_pho_d * BIGVBKF_IGb2_d; % heuristic dynamic
        % measurement update
        BIGVBKF_IGa1_d = 0.5 + BIGVBKF_IGa1_d; % VB step1
        BIGVBKF_IGa2_d = 0.5 + BIGVBKF_IGa2_d; 
        tmpBIGVBKF_BerZt1_d = BIGVBKF_BerZt_d; 
        tmpBIGVBKF_BerZt2_d = BIGVBKF_BerZt_d;
        BIGVBKF_BetaEt1_d = BIGVBKF_BetaE0_d; BIGVBKF_BetaFt1_d = BIGVBKF_BetaF0_d;
        BIGVBKF_BetaEt2_d = BIGVBKF_BetaE0_d; BIGVBKF_BetaFt2_d = BIGVBKF_BetaF0_d;
        tmpBIGVBKF_X_d = BIGVBKF_X_d; tmpBIGVBKF_IGb1_d = BIGVBKF_IGb1_d; 
        tmpBIGVBKF_Px_d = BIGVBKF_Px_d; tmpBIGVBKF_IGb2_d = BIGVBKF_IGb2_d;
        for i = 1 : BIGVBKF_it_max_d % VB iterate dynamic times
%             H = JabH(BIWVBEKF_X_d, SensorsPoint);
            BIGVBKF_X_d_pre = BIGVBKF_X_d;
            if ((tmpBIGVBKF_BerZt1_d > BIGVBKF_BerZt_TH) || (tmpBIGVBKF_BerZt2_d > BIGVBKF_BerZt_TH))
                BIGVBKF_Hk_d = []; BIGVBKF_R_d = [];  YBIGVBKF = [];
                if tmpBIGVBKF_BerZt1_d > BIGVBKF_BerZt_TH 
                    BIGVBKF_Hk_d = [BIGVBKF_Hk_d; H(1,:)]; 
                    YBIGVBKF = [YBIGVBKF; Ypse(1,k)]; 
                    BIGVBKF_R_d = [BIGVBKF_R_d, BIGVBKF_IGb1_d / BIGVBKF_IGa1_d / tmpBIGVBKF_BerZt1_d]; 
                end
                if tmpBIGVBKF_BerZt2_d > BIGVBKF_BerZt_TH 
                    BIGVBKF_Hk_d = [BIGVBKF_Hk_d; H(2,:)]; 
                    YBIGVBKF = [YBIGVBKF; Ypse(2,k)]; 
                    BIGVBKF_R_d = [BIGVBKF_R_d, BIGVBKF_IGb2_d / BIGVBKF_IGa2_d / tmpBIGVBKF_BerZt2_d]; 
                end 
                BIGVBKF_R_d = diag(BIGVBKF_R_d);
                BIGVBKF_Sk_d = BIGVBKF_Hk_d * tmpBIGVBKF_Px_d * BIGVBKF_Hk_d'+ BIGVBKF_R_d; 
                BIGVBKF_Kk_d = tmpBIGVBKF_Px_d * BIGVBKF_Hk_d' / BIGVBKF_Sk_d; 
                BIGVBKF_X_d = tmpBIGVBKF_X_d + BIGVBKF_Kk_d * (YBIGVBKF - BIGVBKF_Hk_d*tmpBIGVBKF_X_d); 
                BIGVBKF_Px_d = tmpBIGVBKF_Px_d - BIGVBKF_Kk_d * BIGVBKF_Sk_d * BIGVBKF_Kk_d'; % VB-E step done 
                if tmpBIGVBKF_BerZt1_d > BIGVBKF_BerZt_TH 
                    BIGVBKF_Bt = (Ypse(1,k) - H(1,:)*BIGVBKF_X_d) * (Ypse(1,k) - H(1,:)*BIGVBKF_X_d)' +  H(1,:) * BIGVBKF_Px_d * H(1,:)';
                    tmpBIGVBKF_BerZt1_d = exp(psi(BIGVBKF_BetaEt1_d) - psi(BIGVBKF_BetaEt1_d + BIGVBKF_BetaFt1_d ) - 0.5 * trace(BIGVBKF_Bt * BIGVBKF_IGa1_d / BIGVBKF_IGb1_d)) / ...
                             (exp(psi(BIGVBKF_BetaEt1_d) - psi(BIGVBKF_BetaEt1_d + BIGVBKF_BetaFt1_d ) - 0.5 * trace(BIGVBKF_Bt * BIGVBKF_IGa1_d / BIGVBKF_IGb1_d)) + ...
                                exp(psi(BIGVBKF_BetaFt1_d ) - psi(BIGVBKF_BetaEt1_d + BIGVBKF_BetaFt1_d )));
                    BIGVBKF_IGb1_d = tmpBIGVBKF_IGb1_d +  0.5 * tmpBIGVBKF_BerZt1_d * (Ypse(1,k) - H(1,:)*BIGVBKF_X_d).^2 + ...
                        0.5 * tmpBIGVBKF_BerZt1_d * diag((H(1,:) * BIGVBKF_Px_d * H(1,:)')); 
                    BIGVBKF_BetaEt1_d = BIGVBKF_BetaE0_d + tmpBIGVBKF_BerZt1_d; 
                    BIGVBKF_BetaFt1_d = BIGVBKF_BetaF0_d + 1 - tmpBIGVBKF_BerZt1_d; % VB-M step done 
                else
                    BIGVBKF_IGb1_d = tmpBIGVBKF_IGb1_d; 
                end
                if tmpBIGVBKF_BerZt2_d > BIGVBKF_BerZt_TH 
                    BIGVBKF_Bt = (Ypse(2,k) - H(2,:)*BIGVBKF_X_d) * (Ypse(2,k) - H(2,:)*BIGVBKF_X_d)' +  H(2,:) * BIGVBKF_Px_d * H(2,:)';
                    tmpBIGVBKF_BerZt2_d = exp(psi(BIGVBKF_BetaEt2_d) - psi(BIGVBKF_BetaEt2_d + BIGVBKF_BetaFt2_d ) - 0.5 * trace(BIGVBKF_Bt * BIGVBKF_IGa2_d / BIGVBKF_IGb2_d)) / ...
                             (exp(psi(BIGVBKF_BetaEt2_d) - psi(BIGVBKF_BetaEt2_d + BIGVBKF_BetaFt2_d ) - 0.5 * trace(BIGVBKF_Bt * BIGVBKF_IGa2_d / BIGVBKF_IGb2_d)) + ...
                                exp(psi(BIGVBKF_BetaFt2_d ) - psi(BIGVBKF_BetaEt2_d + BIGVBKF_BetaFt2_d )));
                    BIGVBKF_IGb2_d = tmpBIGVBKF_IGb2_d +  0.5 * tmpBIGVBKF_BerZt2_d * (Ypse(2,k) - H(2,:)*BIGVBKF_X_d).^2 + ...
                        0.5 * tmpBIGVBKF_BerZt2_d * diag((H(2,:) * BIGVBKF_Px_d * H(2,:)')); 
                    BIGVBKF_BetaEt2_d = BIGVBKF_BetaE0_d + tmpBIGVBKF_BerZt2_d;
                    BIGVBKF_BetaFt2_d = BIGVBKF_BetaF0_d + 1 - tmpBIGVBKF_BerZt2_d; % VB-M step done 
                else
                    BIGVBKF_IGb2_d = tmpBIGVBKF_IGb2_d; 
                end 
                if(norm(BIGVBKF_X_d - BIGVBKF_X_d_pre) < BIGVBKF_E_d)
                    break;
                end
            else
                BIGVBKF_X_d = tmpBIGVBKF_X_d; 
                BIGVBKF_Px_d = tmpBIGVBKF_Px_d; 
                BIGVBKF_IGb1_d = tmpBIGVBKF_IGb1_d; 
                BIGVBKF_IGb2_d = tmpBIGVBKF_IGb2_d; 
                break;
            end
        end
        if tmpBIGVBKF_BerZt1_d <= BIGVBKF_BerZt_TH 
            BIGVBKF_IGa1_d =  BIGVBKF_IGa1_d - 0.5; 
        end
        if tmpBIGVBKF_BerZt2_d <= BIGVBKF_BerZt_TH 
            BIGVBKF_IGa2_d =  BIGVBKF_IGa2_d - 0.5; 
        end
        BIGVBKF_R_d = diag([BIGVBKF_IGb1_d / BIGVBKF_IGa1_d, ...
            BIGVBKF_IGb2_d / BIGVBKF_IGa2_d]); 
        tBIGVBKFEnd = toc;
        resBIGVBKF_it_d(:,k) = i;
        %res
        resBIGVBKF_X_d(:,k) = BIGVBKF_X_d;
        resBIGVBKF_R_d(:,k) = diag(BIGVBKF_R_d);
        resBIGVBZt(:,k) = [tmpBIGVBKF_BerZt1_d; tmpBIGVBKF_BerZt2_d];
        resBIGVBKF_RMSE_d(:,k) = sqrt(trace(BIGVBKF_Px_d));
        resBIGVBKFTime(:,k) = tBIGVBKFEnd;
        k=k+1;
    end   
    %% BIGVBKF(SEQ)
    resBIGVBKFSEQ_X_d = zeros(4, fix(stopT / deltaT));
    resBIGVBKFSEQ_R_d = zeros(2, fix(stopT / deltaT));
    resBIGVBKFSEQ_RMSE_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_1st_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQ_2nd_it_d = zeros(1, fix(stopT / deltaT));
    resBIGVBKFSEQTime = zeros(1, fix(stopT / deltaT));
    resBIGVBSEQZt = zeros(2, fix(stopT / deltaT));
    BIGVBKFSEQ_X_d = filter_X0;
    BIGVBKFSEQ_Px_d = filter_P0;  
    BIGVBKFSEQ_BerZt_d = 1; % Bernoulli param. 
    BIGVBKFSEQ_BerZt_TH = 0.1;
    BIGVBKFSEQ_BetaE0_d = 0.9; BIGVBKFSEQ_BetaF0_d = 0.1; % Beta params.
    BIGVBKFSEQ_IGa1_d = 1; BIGVBKFSEQ_IGb1_d = R(1,1);%BIWVBEKFSEQ_IGb1_d = 50^2; % Inv-Gamma param.
    BIGVBKFSEQ_IGa2_d = 1; BIGVBKFSEQ_IGb2_d = R(2,1);%BIWVBEKFSEQ_IGb2_d = 50^2; % Inv-Gamma param.
    BIGVBKFSEQ_it_max_d = VB_it_Max;
    BIGVBKFSEQ_E_d = VB_D_Err_TH; % error threshold
    BIGVBKFSEQ_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        tic;
        % time update
        BIGVBKFSEQ_X_d =  F*BIGVBKFSEQ_X_d+ B*U(:,k); 
        BIGVBKFSEQ_Px_d = F*BIGVBKFSEQ_Px_d*F'+ Q;
        BIGVBKFSEQ_IGa1_d = BIGVBKFSEQ_pho_d * BIGVBKFSEQ_IGa1_d;
        BIGVBKFSEQ_IGa2_d = BIGVBKFSEQ_pho_d * BIGVBKFSEQ_IGa2_d; % heuristic dynamic
        BIGVBKFSEQ_IGb1_d = BIGVBKFSEQ_pho_d * BIGVBKFSEQ_IGb1_d; 
        BIGVBKFSEQ_IGb2_d = BIGVBKFSEQ_pho_d * BIGVBKFSEQ_IGb2_d; % heuristic dynamic
        % measurement update
        % H dim 1 
        H1 = H(1,:); H2 = H(2,:); 
        BIGVBKFSEQ_IGa1_d = 0.5 + BIGVBKFSEQ_IGa1_d; % VB step1 
        tmpBIGVBKFSEQ_BerZ1t_d = BIGVBKFSEQ_BerZt_d; 
        BIGVBKFSEQ_BetaE1t_d = BIGVBKFSEQ_BetaE0_d;  
        BIGVBKFSEQ_BetaF1t_d = BIGVBKFSEQ_BetaF0_d; 
        tmpBIGVBKFSEQ_X_d = BIGVBKFSEQ_X_d; 
        tmpBIGVBKFSEQ_Px_d = BIGVBKFSEQ_Px_d; 
        tmpBIGVBEKFSEQ_IGb1_d = BIGVBKFSEQ_IGb1_d; 
        % H1
        for i = 1 : BIGVBKFSEQ_it_max_d % VB iterate dynamic times
            BIGVBKFSEQ_Hk1_d = H1;
            if tmpBIGVBKFSEQ_BerZ1t_d > BIGVBKFSEQ_BerZt_TH 
                BIGVBKFSEQ_X_d_pre = BIGVBKFSEQ_X_d; 
                BIGVBKFSEQ_R1_d = BIGVBKFSEQ_IGb1_d / BIGVBKFSEQ_IGa1_d; % VB-E step begin
                BIGVBKFSEQ_Sk_d = BIGVBKFSEQ_Hk1_d * tmpBIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk1_d'+ BIGVBKFSEQ_R1_d / tmpBIGVBKFSEQ_BerZ1t_d; 
                BIGVBKFSEQ_Kk_d = tmpBIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk1_d' / BIGVBKFSEQ_Sk_d; 
                BIGVBKFSEQ_X_d = tmpBIGVBKFSEQ_X_d + BIGVBKFSEQ_Kk_d * (Ypse(1,k) - BIGVBKFSEQ_Hk1_d * tmpBIGVBKFSEQ_X_d); 
                BIGVBKFSEQ_Px_d = tmpBIGVBKFSEQ_Px_d - BIGVBKFSEQ_Kk_d * BIGVBKFSEQ_Sk_d * BIGVBKFSEQ_Kk_d'; % VB-E step done 
                BIGVBKFSEQ_Bt = (Ypse(1,k) - BIGVBKFSEQ_Hk1_d * BIGVBKFSEQ_X_d) * (Ypse(1,k) - BIGVBKFSEQ_Hk1_d * BIGVBKFSEQ_X_d)' + BIGVBKFSEQ_Hk1_d * BIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk1_d'; 
                tmpBIGVBKFSEQ_BerZ1t_d = exp(psi(BIGVBKFSEQ_BetaE1t_d) - psi(BIGVBKFSEQ_BetaE1t_d + BIGVBKFSEQ_BetaF1t_d ) - 0.5 * trace(BIGVBKFSEQ_Bt / BIGVBKFSEQ_R1_d)) / ... 
                    (exp(psi(BIGVBKFSEQ_BetaE1t_d) - psi(BIGVBKFSEQ_BetaE1t_d + BIGVBKFSEQ_BetaF1t_d ) - 0.5 * trace(BIGVBKFSEQ_Bt / BIGVBKFSEQ_R1_d)) + ... 
                    exp(psi(BIGVBKFSEQ_BetaF1t_d ) - psi(BIGVBKFSEQ_BetaE1t_d + BIGVBKFSEQ_BetaF1t_d )));  
                BIGVBKFSEQ_IGb1_d = tmpBIGVBEKFSEQ_IGb1_d + ... % VB-M step begin 
                    0.5 * tmpBIGVBKFSEQ_BerZ1t_d * (Ypse(1,k) - BIGVBKFSEQ_Hk1_d * BIGVBKFSEQ_X_d).^2 + ... 
                    0.5 * tmpBIGVBKFSEQ_BerZ1t_d * diag((BIGVBKFSEQ_Hk1_d*BIGVBKFSEQ_Px_d*BIGVBKFSEQ_Hk1_d')); 
                BIGVBKFSEQ_BetaE1t_d = BIGVBKFSEQ_BetaE0_d + tmpBIGVBKFSEQ_BerZ1t_d; 
                BIGVBKFSEQ_BetaF1t_d = BIGVBKFSEQ_BetaF0_d + 1 - tmpBIGVBKFSEQ_BerZ1t_d; % VB-M step done 
                if(norm(BIGVBKFSEQ_X_d - BIGVBKFSEQ_X_d_pre) < BIGVBKFSEQ_E_d) 
                    break;
                end
            else
                BIGVBKFSEQ_X_d = tmpBIGVBKFSEQ_X_d;
                BIGVBKFSEQ_Px_d = tmpBIGVBKFSEQ_Px_d;
                BIGVBKFSEQ_IGb1_d = tmpBIGVBEKFSEQ_IGb1_d; 
                BIGVBKFSEQ_IGa1_d = BIGVBKFSEQ_IGa1_d - 0.5;
                tmpBIGVBKFSEQ_BerZ1t_d = 0;
                BIGVBKFSEQ_R1_d = BIGVBKFSEQ_IGb1_d / BIGVBKFSEQ_IGa1_d; 
                break;
            end
        end
        resBIGVBKFSEQ_1st_it_d(:,k) = i;
        % H dim 2
        BIGVBKFSEQ_IGa2_d = 0.5 + BIGVBKFSEQ_IGa2_d; % VB step1
        tmpBIGVBKFSEQ_BerZ2t_d = BIGVBKFSEQ_BerZt_d; 
        BIGVBKFSEQ_BetaE2t_d = BIGVBKFSEQ_BetaE0_d; 
        BIGVBKFSEQ_BetaF2t_d = BIGVBKFSEQ_BetaF0_d;
        tmpBIGVBKFSEQ_X_d = BIGVBKFSEQ_X_d; 
        tmpBIGVBKFSEQ_Px_d = BIGVBKFSEQ_Px_d; 
        tmpBIGVBKFSEQ_IGb2_d = BIGVBKFSEQ_IGb2_d; 
        for i = 1 : BIGVBKFSEQ_it_max_d % VB iterate dynamic times
            BIGVBKFSEQ_Hk2_d = H2;
            if tmpBIGVBKFSEQ_BerZ2t_d > BIGVBKFSEQ_BerZt_TH
                BIGVBKFSEQ_X_d_pre = BIGVBKFSEQ_X_d;
                BIGVBKFSEQ_R2_d = BIGVBKFSEQ_IGb2_d / BIGVBKFSEQ_IGa2_d; % VB-E step begin
                BIGVBKFSEQ_Sk_d = BIGVBKFSEQ_Hk2_d * tmpBIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk2_d'+ BIGVBKFSEQ_R2_d / tmpBIGVBKFSEQ_BerZ2t_d;
                BIGVBKFSEQ_Kk_d = tmpBIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk2_d' / BIGVBKFSEQ_Sk_d;
                BIGVBKFSEQ_X_d = tmpBIGVBKFSEQ_X_d + BIGVBKFSEQ_Kk_d * (Ypse(2,k) - BIGVBKFSEQ_Hk2_d * tmpBIGVBKFSEQ_X_d);
                BIGVBKFSEQ_Px_d = tmpBIGVBKFSEQ_Px_d - BIGVBKFSEQ_Kk_d * BIGVBKFSEQ_Sk_d * BIGVBKFSEQ_Kk_d'; % VB-E step done
                BIGVBKFSEQ_Bt = (Ypse(2,k) - BIGVBKFSEQ_Hk2_d * BIGVBKFSEQ_X_d) * (Ypse(2,k) - BIGVBKFSEQ_Hk2_d * BIGVBKFSEQ_X_d)' + BIGVBKFSEQ_Hk2_d * BIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk2_d';
                tmpBIGVBKFSEQ_BerZ2t_d = exp(psi(BIGVBKFSEQ_BetaE2t_d) - psi(BIGVBKFSEQ_BetaE2t_d + BIGVBKFSEQ_BetaF2t_d ) - 0.5 * trace(BIGVBKFSEQ_Bt / BIGVBKFSEQ_R2_d)) / ...
                    (exp(psi(BIGVBKFSEQ_BetaE2t_d) - psi(BIGVBKFSEQ_BetaE2t_d + BIGVBKFSEQ_BetaF2t_d ) - 0.5 * trace(BIGVBKFSEQ_Bt / BIGVBKFSEQ_R2_d)) + ...
                    exp(psi(BIGVBKFSEQ_BetaF2t_d ) - psi(BIGVBKFSEQ_BetaE2t_d + BIGVBKFSEQ_BetaF2t_d )));
                BIGVBKFSEQ_IGb2_d = tmpBIGVBKFSEQ_IGb2_d + ... % VB-M step begin 
                    0.5 * tmpBIGVBKFSEQ_BerZ2t_d * (Ypse(2,k) - BIGVBKFSEQ_Hk2_d * BIGVBKFSEQ_X_d).^2 + ... 
                    0.5 * tmpBIGVBKFSEQ_BerZ2t_d * diag((BIGVBKFSEQ_Hk2_d * BIGVBKFSEQ_Px_d * BIGVBKFSEQ_Hk2_d')); 
                BIGVBKFSEQ_BetaE2t_d = BIGVBKFSEQ_BetaE0_d + tmpBIGVBKFSEQ_BerZ2t_d; 
                BIGVBKFSEQ_BetaF2t_d = BIGVBKFSEQ_BetaF0_d + 1 - tmpBIGVBKFSEQ_BerZ2t_d; % VB-M step done
                if(norm(BIGVBKFSEQ_X_d - BIGVBKFSEQ_X_d_pre) < BIGVBKFSEQ_E_d)
                    break;
                end
            else
                BIGVBKFSEQ_X_d = tmpBIGVBKFSEQ_X_d; 
                BIGVBKFSEQ_Px_d = tmpBIGVBKFSEQ_Px_d; 
                BIGVBKFSEQ_IGb2_d = tmpBIGVBKFSEQ_IGb2_d; 
                BIGVBKFSEQ_IGa2_d = BIGVBKFSEQ_IGa2_d - 0.5; 
                tmpBIGVBKFSEQ_BerZ2t_d = 0;
                BIGVBKFSEQ_R2_d = BIGVBKFSEQ_IGb2_d / BIGVBKFSEQ_IGa2_d; 
                break;
            end
        end
        tBIGVBKFSEQEnd = toc;
        resBIGVBKFSEQ_2nd_it_d(:,k) = i;
        %res
        resBIGVBKFSEQ_it_d = resBIGVBKFSEQ_1st_it_d + resBIGVBKFSEQ_2nd_it_d;
        resBIGVBKFSEQ_X_d(:,k) = BIGVBKFSEQ_X_d;
        resBIGVBKFSEQ_R_d(:,k) = [BIGVBKFSEQ_R1_d; BIGVBKFSEQ_R2_d];
        resBIGVBSEQZt(:,k) = [tmpBIGVBKFSEQ_BerZ1t_d; tmpBIGVBKFSEQ_BerZ2t_d];
        resBIGVBKFSEQ_RMSE_d(:,k) = sqrt(trace(BIGVBKFSEQ_Px_d));
        resBIGVBKFSEQTime(:,k) = tBIGVBKFSEQEnd;
        k=k+1;
    end 
    %% Evaluate
    % CRLB of model
    if mc == 1
        resCRLB_trLB = zeros(1, fix(stopT/deltaT));
        CRLB_J = inv(filter_P0);
        k=1;
        for t = 0:deltaT:stopT-deltaT
            CRLB_D11 = (F'/Q)*F;
            CRLB_D12 = -F'/Q;
            CRLB_D22 = inv(Q) + (H'/(diag(R(:,k))))*H;
            CRLB_J = CRLB_D22 - (CRLB_D12'/(CRLB_J + CRLB_D11))*CRLB_D12;
            %res
            resCRLB_trLB(:,k) = sqrt(trace(inv(CRLB_J)));
            k=k+1;
        end
    end
    % MC KF result
    RMSE_KF_X(mc,1) = rmse(resKF_X(1,:),Xpse(1,:));
    RMSE_KF_X(mc,2) = rmse(resKF_X(2,:),Xpse(2,:));
    RMSE_KF_X(mc,3) = rmse(resKF_X(3,:),Xpse(3,:));
    RMSE_KF_X(mc,4) = rmse(resKF_X(4,:),Xpse(4,:));
    % MC VBKF dynamic iteration result
    RMSE_VBKF_X_d(mc,1) = rmse(resVBKF_X_d(1,:),Xpse(1,:)); % state result
    RMSE_VBKF_X_d(mc,2) = rmse(resVBKF_X_d(2,:),Xpse(2,:));
    RMSE_VBKF_X_d(mc,3) = rmse(resVBKF_X_d(3,:),Xpse(3,:));
    RMSE_VBKF_X_d(mc,4) = rmse(resVBKF_X_d(4,:),Xpse(4,:)); 
    RMSE_VBKF_R_d(mc,1) = rmse(resVBKF_R_d(1,:),R(1, :)); % measurement noise statistics result
    RMSE_VBKF_R_d(mc,2) = rmse(resVBKF_R_d(2,:),R(2, :));    
    % MC RSTKF result
    RMSE_RSTKF_X(mc, 1) = rmse(resRSTKF_X(1,:), Xpse(1, :));
    RMSE_RSTKF_X(mc, 2) = rmse(resRSTKF_X(2,:), Xpse(2, :));
    RMSE_RSTKF_X(mc, 3) = rmse(resRSTKF_X(3,:), Xpse(3, :));
    RMSE_RSTKF_X(mc, 4) = rmse(resRSTKF_X(4,:), Xpse(4, :));
    % MC new_aprivbkf result
    RMSE_NEWAPRIVBKF_X(mc, 1) = rmse(resNew_aprivbkf_X(1,:), Xpse(1, :));
    RMSE_NEWAPRIVBKF_X(mc, 2) = rmse(resNew_aprivbkf_X(2,:), Xpse(2, :));
    RMSE_NEWAPRIVBKF_X(mc, 3) = rmse(resNew_aprivbkf_X(3,:), Xpse(3, :));
    RMSE_NEWAPRIVBKF_X(mc, 4) = rmse(resNew_aprivbkf_X(4,:), Xpse(4, :));
    RMSE_NEWAPRIVBKF_R(mc, 1) = rmse(resNew_aprivbkf_R(1,:), R(1, :));
    RMSE_NEWAPRIVBKF_R(mc, 2) = rmse(resNew_aprivbkf_R(2,:), R(2, :));
    % MC swrkf result
    RMSE_SWRKF_X(mc, 1) = rmse(resSwrkf_X(1,:), Xpse(1, :));
    RMSE_SWRKF_X(mc, 2) = rmse(resSwrkf_X(2,:), Xpse(2, :));
    RMSE_SWRKF_X(mc, 3) = rmse(resSwrkf_X(3,:), Xpse(3, :));
    RMSE_SWRKF_X(mc, 4) = rmse(resSwrkf_X(4,:), Xpse(4, :));
    RMSE_SWRKF_R(mc, 1) = rmse(resSwrkf_R(1,:), R(1, :));
    RMSE_SWRKF_R(mc, 2) = rmse(resSwrkf_R(2,:), R(2, :));
    % MC ssmkf result
    RMSE_SSMKF_X(mc, 1) = rmse(resSsmkf_X(1,:), Xpse(1, :));
    RMSE_SSMKF_X(mc, 2) = rmse(resSsmkf_X(2,:), Xpse(2, :));
    RMSE_SSMKF_X(mc, 3) = rmse(resSsmkf_X(3,:), Xpse(3, :));
    RMSE_SSMKF_X(mc, 4) = rmse(resSsmkf_X(4,:), Xpse(4, :));
    % MC mckf result
    RMSE_MCKF_X(mc, 1) = rmse(resMCKF_X(1,:), Xpse(1, :));
    RMSE_MCKF_X(mc, 2) = rmse(resMCKF_X(2,:), Xpse(2, :));
    RMSE_MCKF_X(mc, 3) = rmse(resMCKF_X(3,:), Xpse(3, :));
    RMSE_MCKF_X(mc, 4) = rmse(resMCKF_X(4,:), Xpse(4, :));
    % MC BIWVBEKF result
    RMSE_BIGVBKF_X(mc, 1) = rmse(resBIGVBKF_X_d(1, :), Xpse(1, :));
    RMSE_BIGVBKF_X(mc, 2) = rmse(resBIGVBKF_X_d(2, :), Xpse(2, :));
    RMSE_BIGVBKF_X(mc, 3) = rmse(resBIGVBKF_X_d(3, :), Xpse(3, :));
    RMSE_BIGVBKF_X(mc, 4) = rmse(resBIGVBKF_X_d(4, :), Xpse(4, :));
    RMSE_BIGVBKF_R(mc, 1) = rmse(resBIGVBKF_R_d(1, :), R(1, :));
    RMSE_BIGVBKF_R(mc, 2) = rmse(resBIGVBKF_R_d(2, :), R(2, :));
    % MC BIWVBEKFSEQ result
    RMSE_BIGVBKFSEQ_X(mc, 1) = rmse(resBIGVBKFSEQ_X_d(1, :), Xpse(1, :));
    RMSE_BIGVBKFSEQ_X(mc, 2) = rmse(resBIGVBKFSEQ_X_d(2, :), Xpse(2, :));
    RMSE_BIGVBKFSEQ_X(mc, 3) = rmse(resBIGVBKFSEQ_X_d(3, :), Xpse(3, :));
    RMSE_BIGVBKFSEQ_X(mc, 4) = rmse(resBIGVBKFSEQ_X_d(4, :), Xpse(4, :));
    RMSE_BIGVBKFSEQ_R(mc, 1) = rmse(resBIGVBKFSEQ_R_d(1, :), R(1, :));
    RMSE_BIGVBKFSEQ_R(mc, 2) = rmse(resBIGVBKFSEQ_R_d(2, :), R(2, :));
    % MC Time result
    MCCHIKFSEQTime(mc) = mean(resSEQKFCHITime);
    MCVBKFTime(mc) = mean(resVBKFTime);
    MCRSTKFTime(mc) = mean(resRSTKFTime);
    MCNew_aprivbkfTime(mc) = mean(resNew_aprivbkfTime);
    MCSwrkfTime(mc) = mean(resSwrkfTime);
    MCSsmkfTime(mc) = mean(resSsmkfTime);
    MCMckfTime(mc) = mean(resMCKFTime);
    MCBIGVBKFTime(mc) = mean(resBIGVBKFTime);
    MCBIGVBKFSEQTime(mc) = mean(resBIGVBKFSEQTime);
end
%% figures
% plot state
f1 = figure('Name','f1_estimated states');
f1.Units = "inches";
f1.Position = [5.833333333333333,2.979166666666667,11.6667,4.375];
subplot(2,5,[1,2,6,7]);hold on;
plot(Xpse(1,:),Xpse(2,:),'LineWidth',2);
plot(resKF_X(1,:),resKF_X(2,:),'LineWidth',2);
plot(resVBKF_X_d(1,:),resVBKF_X_d(2,:),'LineWidth',2);
plot(resBIGVBKF_X_d(1,:),resBIGVBKF_X_d(2,:),'LineWidth',2);
plot(resBIGVBKFSEQ_X_d(1,:),resBIGVBKFSEQ_X_d(2,:),'LineWidth',2);
plot(Ypse(1,:),Ypse(2,:),'*','MarkerSize',2);
plot(X0(1),X0(2),'^');
plot(Xpse(1,end),Xpse(2,end),'o');
legend('True','chiKF','VBKF','BIGVBKF','BIGVBKFSEQ','Observer','start','end');
set(ylabel(['$p_E$ ($m$)'],'Interpreter','latex'));
set(xlabel(['$p_N$ ($m$)'],'Interpreter','latex'));
subplot(2,5,[3,4,8,9]);hold on;
plot(Xpse(1,:),Xpse(2,:),'LineWidth',2);
plot(resNew_aprivbkf_X(1,:),resNew_aprivbkf_X(2,:),'LineWidth',2);
plot(resSwrkf_X(1,:),resSwrkf_X(2,:),'LineWidth',2);
plot(resSsmkf_X(1,:),resSsmkf_X(2,:),'LineWidth',2);
plot(resRSTKF_X(1,:),resRSTKF_X(2,:),'LineWidth',2);
plot(resMCKF_X(1,:),resMCKF_X(2,:),'LineWidth',2);
plot(resBIGVBKF_X_d(1,:),resBIGVBKF_X_d(2,:),'LineWidth',2);
plot(resBIGVBKFSEQ_X_d(1,:),resBIGVBKFSEQ_X_d(2,:),'LineWidth',2);
plot(Ypse(1,:),Ypse(2,:),'*','MarkerSize',2);
plot(X0(1),X0(2),'^');
plot(Xpse(1,end),Xpse(2,end),'o');
legend('True','new\_aprivbkf','swrkf',...
    'ssmkf','rstkf','mckf','BIGVBKF','BIGVBKFSEQ','Observer','start','end');
set(ylabel(['$p_E$ ($m$)'],'Interpreter','latex'));
set(xlabel(['$p_N$ ($m$)'],'Interpreter','latex'));
subplot(2,5,5);hold on;
plot(Time,[Xpse(3,:);resKF_X(3,:);resVBKF_X_d(3,:);...
    resRSTKF_X(3,:);resNew_aprivbkf_X(3,:);resSwrkf_X(3,:);...
    resSsmkf_X(3,:);resMCKF_X(3,:);resBIGVBKF_X_d(3,:);resBIGVBKFSEQ_X_d(3,:)]');
set(ylabel(['$v_E$ ($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(2,5,10);hold on;
plot(Time,[Xpse(4,:);resKF_X(4,:);resVBKF_X_d(4,:);...
    resRSTKF_X(4,:);resNew_aprivbkf_X(4,:);resSwrkf_X(4,:);...
    resSsmkf_X(4,:);resMCKF_X(4,:);resBIGVBKF_X_d(4,:);resBIGVBKFSEQ_X_d(4,:)]');
set(ylabel(['$v_N$ ($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
sgtitle("State Estimation Result",'Fontsize',10,'Interpreter','Latex');
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


% plot measurement noise statistics
f2 = figure('Name','f2_Measurement Covariance Estimation');
subplot(211);
plot(Time,[R(1,:);resVBKF_R_d(1,:);...
    resNew_aprivbkf_R(1,:); resSwrkf_R(1,:);resBIGVBKF_R_d(1,:);...
    resBIGVBKFSEQ_R_d(1,:)]');
set(ylabel(['$R_1$ ($m^2$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(212);
plot(Time,[R(2,:);resVBKF_R_d(2,:);...
    resNew_aprivbkf_R(2,:); resSwrkf_R(2,:);resBIGVBKF_R_d(2,:); ...
    resBIGVBKFSEQ_R_d(2,:)]');
legend('True','VBKF','new\_aprivbkf','swrkf','BIGVBKF','BIGVBKFSEQ');
set(ylabel(['$R_2$ ($m^2$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
sgtitle("Measurement Covariance Estimation Result",'Fontsize',10,'Interpreter','Latex');
pos = get(f2,'Position');
set(f2,'Units','Inches');
set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

% plot CRLB and RMSE
f3 = figure('Name','f4_CRLB and RMSE');
semilogy(Time,[resKF_RMSE;resVBKF_RMSE_d;resRSTKF_RMSE;...
    resNew_aprivbkf_RMSE;resSwrkf_RMSE;resMCKF_RMSE;resSsmkf_RMSE;...
    resBIGVBKF_RMSE_d;resBIGVBKFSEQ_RMSE_d; resCRLB_trLB;]');
legend('chiKFSEQ','VBKF','rstkf','new\_aprivbkf','swrkf','mckf','ssmkf',...
    'BIGVBKF','BIGVBKFSEQ','CRLB');
set(ylabel(['RMSE error'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
title("CRLB and RMSE",'Fontsize',10,'Interpreter','Latex');

f4 = figure('Name','f6_Bern Zt index');
subplot(3, 1, 1);
p = pcolor(Time,[1,2,3],[resBIGVBZt;zeros(size(resSEQKFCHIIndex(1,:)))]);
set(ylabel('BIGVBEKF','Interpreter','latex'));
subplot(3, 1, 2);
pcolor(Time,[1,2,3],[resBIGVBSEQZt;zeros(size(resSEQKFCHIIndex(1,:)))]);
set(ylabel('BIGVBEKFSEQ','Interpreter','latex'));
subplot(3, 1, 3);
pcolor(Time,[1,2,3],[resSEQKFCHIIndex;zeros(size(resSEQKFCHIIndex(1,:)))]);
set(ylabel('ChiEKF','Interpreter','latex'));
set(xlabel('Time($s$)','Interpreter','latex'));
sgtitle("Bernoulli Zt index  (yellow - accept, blue - reject)",'Fontsize',10,'Interpreter','Latex');
pos = get(f4,'Position');
set(f4,'Units','Inches');
set(f4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

% plot MC RMSE
if(MC_times > 1)

    f5=figure;
    subplot(2, 1, 1);
    semilogy(Time,[resSEQKFCHITime;resVBKFTime;resNew_aprivbkfTime;...
        resSwrkfTime;resSsmkfTime;resRSTKFTime;resMCKFTime;resBIGVBKFTime;resBIGVBKFSEQTime]);
    legend('chiKFSEQ','VBKF','new\_aprivbkf','swrkf','ssmkf','rstkf','mckf',...
        'BIGVBKF','BIGVBKFSEQ');
    set(xlabel(['Time($s$)'],'Interpreter','latex'));
    set(ylabel('Elapsed Time($s$)','Interpreter','latex'));
    subplot(2, 1, 2);
    timeItems = categorical({'chiKFSEQ','VBKF','new\_aprivbkf',...
        'swrkf','ssmkf','rstkf','mckf','BIGVBKF','BIGVBKFSEQ'});
    timeItems = reordercats(timeItems,{'chiKFSEQ','VBKF','new\_aprivbkf',...
        'swrkf','ssmkf','rstkf','mckf','BIGVBKF','BIGVBKFSEQ'}); 
    meanTime = mean([MCCHIKFSEQTime,MCVBKFTime,MCNew_aprivbkfTime,...
        MCSwrkfTime,MCSsmkfTime,MCRSTKFTime,MCMckfTime,MCBIGVBKFTime,MCBIGVBKFSEQTime]);
    b=bar(timeItems,meanTime','FaceColor','flat');
    set(ylabel('Monte-Carlo Elapsed Time($s$)','Interpreter','latex'));
    xtips = b.XEndPoints;
    ytips = b.YEndPoints+0.0000;
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
    items={'chiKFSEQ','VBKF','rstkf','new_aprivbkf','swrkf','ssmkf','mckf',...
        'BIGVBKF','BIGVBKFSEQ'};
    % state MC RMSE
    f6 = figure('Name','f5_state MC RMSE');
    subplot(221);boxplot([RMSE_KF_X(:,1),RMSE_VBKF_X_d(:,1),...
        RMSE_RSTKF_X(:,1), RMSE_NEWAPRIVBKF_X(:,1), ...
        RMSE_SWRKF_X(:,1),RMSE_SSMKF_X(:,1),RMSE_MCKF_X(:,1), ...
        RMSE_BIGVBKF_X(:,1),RMSE_BIGVBKFSEQ_X(:,1)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log'; ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Position $u$'],'Interpreter','latex'));
    set(ylabel(['Position $x_1$'],'Interpreter','latex'));
    subplot(222);boxplot([RMSE_KF_X(:,2),RMSE_VBKF_X_d(:,2),...
        RMSE_RSTKF_X(:,2), RMSE_NEWAPRIVBKF_X(:,2), ...
        RMSE_SWRKF_X(:,2),RMSE_SSMKF_X(:,2),RMSE_MCKF_X(:,2), ...
        RMSE_BIGVBKF_X(:,2),RMSE_BIGVBKFSEQ_X(:,2)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Position $v$'],'Interpreter','latex'));
    set(ylabel(['Position $x_2$'],'Interpreter','latex'));
    subplot(223);boxplot([RMSE_KF_X(:,3),RMSE_VBKF_X_d(:,3),...
        RMSE_RSTKF_X(:,3), RMSE_NEWAPRIVBKF_X(:,3), ...
        RMSE_SWRKF_X(:,3),RMSE_SSMKF_X(:,3),RMSE_MCKF_X(:,3), ...
        RMSE_BIGVBKF_X(:,3),RMSE_BIGVBKFSEQ_X(:,3)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca;ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Velocity $u$'],'Interpreter','latex'));
    set(ylabel(['Velocity $x_3$'],'Interpreter','latex'));
    subplot(224);boxplot([RMSE_KF_X(:,4),RMSE_VBKF_X_d(:,4),...
        RMSE_RSTKF_X(:,4), RMSE_NEWAPRIVBKF_X(:,4), ...
        RMSE_SWRKF_X(:,4),RMSE_SSMKF_X(:,4),RMSE_MCKF_X(:,4), ...
        RMSE_BIGVBKF_X(:,4),RMSE_BIGVBKFSEQ_X(:,4)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    %     set(ylabel(['Velocity $v$'],'Interpreter','latex'));
    set(ylabel(['Velocity $x_4$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo state RMSE Value",'Fontsize',10,'Interpreter','Latex');
    pos = get(f6,'Position');
    set(f6,'Units','Inches');
    set(f6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

    % measurement noise statistics MC RMSE
    items={'VBKF','new_aprivbkf','swrkf','BIGVBKF','BIGVBKFSEQ'};
    f7 = figure('Name','f6_measurement noise statistics MC RMSE');
    subplot(211);boxplot([RMSE_VBKF_R_d(:,1),...
        RMSE_NEWAPRIVBKF_R(:,1), RMSE_SWRKF_R(:,1), RMSE_BIGVBKF_R(:,1), ...
        RMSE_BIGVBKFSEQ_R(:,1)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$R_{t1}$'],'Interpreter','latex'));
    ax = gca;ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    subplot(212);boxplot([RMSE_VBKF_R_d(:,2), ...
        RMSE_NEWAPRIVBKF_R(:,2), RMSE_SWRKF_R(:,2), RMSE_BIGVBKF_R(:,2),...
        RMSE_BIGVBKFSEQ_R(:,2)],items,"Colors",C,'PlotStyle','compact','LabelOrientation','horizontal');
    set(ylabel(['$R_{t2}$'],'Interpreter','latex'));
    ax = gca; ax.YScale = 'log';ax.YGrid = 'on';ax.YMinorGrid = 'on';
    sgtitle("Monte-Carlo measurement noise RMSE Value",'Fontsize',10,'Interpreter','Latex');
    pos = get(f7,'Position');
    set(f7,'Units','Inches');
    set(f7,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);

end
