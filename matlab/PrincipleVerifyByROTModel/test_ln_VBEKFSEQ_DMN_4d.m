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

%% Inv-Gamma 4 dimension Dynamic Measurement Noise
% Yulu Zhong
% 4 state variables u, v, dot{u}, dot{v}
% linear observer
% U controler by acc. dot{dot{u}}, dot{dot{v}}
close all;
clear;
% true trajectory
deltaT = 0.1; % control value 0.01 sec.
stopT = 50; % 
X0 = [-20,8,0,0]'; % initial vel. 0 pos. (-20,8)
U = zeros(2, fix(stopT/deltaT)); % set acc.
k=1;
for t = 0:deltaT:stopT
    if(t < 3)
        U(1,k) = 3; % speed up from u dir.
    elseif(t <4)
         U(1,k) = 0;% uniform
    elseif(t<5)
        U(1,k) = -9;
        U(2,k) = -8;% turn right
    elseif(t<6)
        U(2,k) = 4; 
    elseif(t<7)
        U(1,k) = 0;
    end
    k = k+1;
end


Xpse = zeros(4, fix(stopT/deltaT)); % prealloc true X
Ypse = zeros(2, fix(stopT/deltaT)); % prealloc true Y
Time = zeros(1, fix(stopT/deltaT));
Xtrue(:,1) = X0;
Xpse(:,1) = X0;

F = [eye(2),deltaT*eye(2);
    zeros(2,2),eye(2)];
B = [0.5*deltaT^2*eye(2);
    deltaT*eye(2)];
H = [eye(2),zeros(2,2)];
Q = [deltaT^3/3*eye(2),deltaT^2/2*eye(2);
     deltaT^2/2*eye(2), deltaT*eye(2)];
nominal_R =  0.64*eye(2);
R = 0.64*eye(2); 
TV_factor_of_R = zeros(1, fix(stopT/deltaT)); % time-varying factor of R
k=1;
for t = 0:deltaT:stopT-deltaT
    TV_factor_of_R(k) = 2 - cos(0.05*pi*t);
    Xpse(:,k+1) = F*Xpse(:,k)+ B*U(:,k) + chol(Q)*randn(4,1);
    Ypse(:,k) = H*Xpse(:,k+1)+chol(TV_factor_of_R(k)*R)*randn(2,1);
    Time(k) = t;
    k = k+1;
end
Xpse = Xpse(:,1:end-1);

MC_times = 100; % Monte-Carlo times
RMSE_KF_X = zeros(MC_times,4); % prealloc MC result
RMSE_VBKF_X_f = zeros(MC_times,4); % prealloc MC result
RMSE_VBKF_R_f = zeros(MC_times,2); % prealloc MC result
RMSE_VBKF_X_d = zeros(MC_times,4); % prealloc MC result
RMSE_VBKF_R_d = zeros(MC_times,2); % prealloc MC result
RMSE_VBKFSEQ_X_d = zeros(MC_times,4); % prealloc MC result
RMSE_VBKFSEQ_R_d = zeros(MC_times,2); % prealloc MC result
for mc = 1:MC_times
    filter_P0 = 200*Q;
    filter_X0 = X0 + chol(filter_P0)*randn(4,1);
    VB_D_Err_TH = 1e-3;
    %% KF
    resKF_X = zeros(4, fix(stopT/deltaT));
    KF_Px = filter_P0;
    KF_X = filter_X0;
    KF_R = nominal_R;
%     KF_Px = diag([1,1,0.001,0.001]);
    resKF_RMSE = zeros(1, fix(stopT/deltaT));
    k=1;
    for t = 0:deltaT:stopT-deltaT
        % time update
        KF_X = F*KF_X+ B*U(:,k);
        KF_Px = F*KF_Px*F'+ Q;
        % measurement update
        KF_Hk = H;
        KF_Sk = KF_Hk*KF_Px*KF_Hk'+ TV_factor_of_R(k)*KF_R;
        KF_Kk = KF_Px*KF_Hk'/KF_Sk;
        KF_X = KF_X + KF_Kk*(Ypse(:,k) - KF_Hk*KF_X);
        KF_Px = KF_Px - KF_Kk*KF_Sk*KF_Kk';
        %res
        resKF_X(:,k) = KF_X;
        resKF_RMSE(:,k) = sqrt(trace(KF_Px));
        k=k+1;
    end
    %% VBKF - InvGamma fixed iteration
    resVBKF_X_f = zeros(4, fix(stopT/deltaT));
    resVBKF_R_f = zeros(2, fix(stopT/deltaT));
    resVBKF_RMSE_f = zeros(1, fix(stopT/deltaT));
    VBKF_X_f = filter_X0;
    VBKF_Px_f = filter_P0;  
    VBKF_IGa1_f = 1;VBKF_IGb1_f = 1; % Inv-Gamma param.
    VBKF_IGa2_f = 1;VBKF_IGb2_f = 1; % Inv-Gamma param.
    VBKF_it_f = 4; % iteration times
    VBKF_pho_f = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        % time update
        VBKF_X_f = F*VBKF_X_f+ B*U(:,k);
        VBKF_Px_f = F*VBKF_Px_f*F'+ Q;
        VBKF_IGa1_f = VBKF_pho_f * VBKF_IGa1_f; 
        VBKF_IGa2_f = VBKF_pho_f * VBKF_IGa2_f; % heuristic dynamic
        VBKF_IGb1_f = VBKF_pho_f * VBKF_IGb1_f; 
        VBKF_IGb2_f = VBKF_pho_f * VBKF_IGb2_f; % heuristic dynamic
        % measurement update
        VBKF_Hk_f = H;
        VBKF_IGa1_f = 0.5 + VBKF_IGa1_f; % VB step1
        VBKF_IGa2_f = 0.5 + VBKF_IGa2_f; 
        tmpVBKF_X_f = VBKF_X_f; tmpVBKF_IGb1_f = VBKF_IGb1_f; 
        tmpVBKF_Px_f = VBKF_Px_f; tmpVBKF_IGb2_f = VBKF_IGb2_f;
        for i = 1 : VBKF_it_f % VB iterate fixed times
            VBKF_R_f = diag([VBKF_IGb1_f/VBKF_IGa1_f,VBKF_IGb2_f/VBKF_IGa2_f]); % VB-E step begin
            VBKF_Sk_f = VBKF_Hk_f*tmpVBKF_Px_f*VBKF_Hk_f'+ VBKF_R_f;
            VBKF_Kk_f = tmpVBKF_Px_f*VBKF_Hk_f'/VBKF_Sk_f;
            VBKF_X_f = tmpVBKF_X_f + VBKF_Kk_f*(Ypse(:,k) - VBKF_Hk_f*tmpVBKF_X_f);
            VBKF_Px_f = tmpVBKF_Px_f - VBKF_Kk_f*VBKF_Sk_f*VBKF_Kk_f'; % VB-E step done
            tmp = [tmpVBKF_IGb1_f; tmpVBKF_IGb2_f]+ ... % VB-M step begin
                0.5*(Ypse(:,k) - VBKF_Hk_f*VBKF_X_f).^2 + ...
                0.5*diag((VBKF_Hk_f*VBKF_Px_f*VBKF_Hk_f')); 
            VBKF_IGb1_f = tmp(1); VBKF_IGb2_f = tmp(2); % VB-M step done
        end
        %res
        resVBKF_X_f(:,k) = VBKF_X_f;
        resVBKF_R_f(:,k) = diag(VBKF_R_f);
        resVBKF_RMSE_f(:,k) = sqrt(trace(VBKF_Px_f));
        k=k+1;
    end    
    %% VBKF  - InvGamma dynamic iteration
    resVBKF_X_d = zeros(4, fix(stopT/deltaT));
    resVBKF_R_d = zeros(2, fix(stopT/deltaT));
    resVBKF_RMSE_d = zeros(1, fix(stopT/deltaT));
    resVBKF_it_d = zeros(1, fix(stopT/deltaT));
    VBKF_X_d = filter_X0;
    VBKF_Px_d = filter_P0;  
    VBKF_IGa1_d = 1;VBKF_IGb1_d = 1; % Inv-Gamma param.
    VBKF_IGa2_d = 1;VBKF_IGb2_d = 1; % Inv-Gamma param.
    VBKF_it_max_d = 20; % iteration times
    VBKF_E_d = VB_D_Err_TH; % error threshold
    VBKF_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
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
        resVBKF_it_d(:,k) = i;
        %res
        resVBKF_X_d(:,k) = VBKF_X_d;
        resVBKF_R_d(:,k) = diag(VBKF_R_d);
        resVBKF_RMSE_d(:,k) = sqrt(trace(VBKF_Px_d));
        k=k+1;
    end        
    %% VBKFSEQ - InvGamma dynamic iteration
    resVBKFSEQ_X_d = zeros(4, fix(stopT/deltaT));
    resVBKFSEQ_R_d = zeros(2, fix(stopT/deltaT));
    resVBKFSEQ_RMSE_d = zeros(1, fix(stopT/deltaT));
    resVBKFSEQ_1st_it_d = zeros(1, fix(stopT/deltaT));
    resVBKFSEQ_2nd_it_d = zeros(1, fix(stopT/deltaT));
    resVBKFSEQ_it_d = zeros(1, fix(stopT/deltaT));
    VBKFSEQ_X_d = filter_X0;
    VBKFSEQ_Px_d = filter_P0;  
    VBKFSEQ_IGa1_d = 1;VBKFSEQ_IGb1_d = 1; % Inv-Gamma param.
    VBKFSEQ_IGa2_d = 1;VBKFSEQ_IGb2_d = 1; % Inv-Gamma param.
    VBKFSEQ_it_max_d = 20; % iteration times
    VBKFSEQ_E_d = VB_D_Err_TH; % error threshold
    VBKFSEQ_pho_d = 1 - exp(-4);
    k=1;
    for t = 0:deltaT:stopT-deltaT
        % time update
        VBKFSEQ_X_d = F*VBKFSEQ_X_d+ B*U(:,k);
        VBKFSEQ_Px_d = F*VBKFSEQ_Px_d*F'+ Q;
        VBKFSEQ_IGa1_d = VBKFSEQ_pho_d * VBKFSEQ_IGa1_d; 
        VBKFSEQ_IGa2_d = VBKFSEQ_pho_d * VBKFSEQ_IGa2_d; % heuristic dynamic
        VBKFSEQ_IGb1_d = VBKF_pho_d * VBKFSEQ_IGb1_d; 
        VBKFSEQ_IGb2_d = VBKFSEQ_pho_d * VBKFSEQ_IGb2_d; % heuristic dynamic
        % measurement update
        VBKFSEQ_Hk_d = H;
        VBKFSEQ_IGa1_d = 0.5 + VBKFSEQ_IGa1_d; % VB step1
        VBKFSEQ_IGa2_d = 0.5 + VBKFSEQ_IGa2_d; 
        tmpVBKFSEQ_X_d = VBKFSEQ_X_d; tmpVBKFSEQ_IGb1_d = VBKFSEQ_IGb1_d; 
        tmpVBKFSEQ_Px_d = VBKFSEQ_Px_d; tmpVBKFSEQ_IGb2_d = VBKFSEQ_IGb2_d;
        for i = 1 : VBKFSEQ_it_max_d % VB iterate dynamic times
            VBKFSEQ_X_d_pre = VBKFSEQ_X_d;
            VBKFSEQ_R1_d = VBKFSEQ_IGb1_d/VBKFSEQ_IGa1_d; % VB-E step begin
            VBKFSEQ_Sk_d = VBKFSEQ_Hk_d(1,:)*tmpVBKFSEQ_Px_d*VBKFSEQ_Hk_d(1,:)'+ VBKFSEQ_R1_d;
            VBKFSEQ_Kk_d = tmpVBKFSEQ_Px_d*VBKFSEQ_Hk_d(1,:)'/VBKFSEQ_Sk_d;
            VBKFSEQ_X_d = tmpVBKFSEQ_X_d + VBKFSEQ_Kk_d*(Ypse(1,k) - VBKFSEQ_Hk_d(1,:)*tmpVBKFSEQ_X_d);
            VBKFSEQ_Px_d = tmpVBKFSEQ_Px_d - VBKFSEQ_Kk_d*VBKFSEQ_Sk_d*VBKFSEQ_Kk_d'; % VB-E step done
            VBKFSEQ_IGb1_d = tmpVBKFSEQ_IGb1_d+ ... % VB-M step begin
                0.5*(Ypse(1,k) - VBKFSEQ_Hk_d(1,:)*VBKFSEQ_X_d).^2 + ...
                0.5*(VBKFSEQ_Hk_d(1,:)*VBKFSEQ_Px_d*VBKFSEQ_Hk_d(1,:)'); % VB-M step done
            if(norm(VBKFSEQ_X_d-VBKFSEQ_X_d_pre) < VBKFSEQ_E_d)
                break;
            end
        end
        resVBKFSEQ_1st_it_d(:,k) = i;
        %test1
        tmpVBKFSEQ_X_d = VBKFSEQ_X_d; %tmpVBKFSEQ_IGb1_d = VBKFSEQ_IGb1_d; 
        tmpVBKFSEQ_Px_d = VBKFSEQ_Px_d;% tmpVBKFSEQ_IGb2_d = VBKFSEQ_IGb2_d;
        for i = 1 : VBKFSEQ_it_max_d % VB iterate dynamic times
            VBKFSEQ_X_d_pre = VBKFSEQ_X_d;
            VBKFSEQ_R2_d = VBKFSEQ_IGb2_d/VBKFSEQ_IGa2_d; % VB-E step begin
            VBKFSEQ_Sk_d = VBKFSEQ_Hk_d(2,:)*tmpVBKFSEQ_Px_d*VBKFSEQ_Hk_d(2,:)'+ VBKFSEQ_R2_d;
            VBKFSEQ_Kk_d = tmpVBKFSEQ_Px_d*VBKFSEQ_Hk_d(2,:)'/VBKFSEQ_Sk_d;
            VBKFSEQ_X_d = tmpVBKFSEQ_X_d + VBKFSEQ_Kk_d*(Ypse(2,k) - VBKFSEQ_Hk_d(2,:)*tmpVBKFSEQ_X_d);
            VBKFSEQ_Px_d = tmpVBKFSEQ_Px_d - VBKFSEQ_Kk_d*VBKFSEQ_Sk_d*VBKFSEQ_Kk_d'; % VB-E step done
            VBKFSEQ_IGb2_d = tmpVBKFSEQ_IGb2_d+ ... % VB-M step begin
                0.5*(Ypse(2,k) - VBKFSEQ_Hk_d(2,:)*VBKFSEQ_X_d).^2 + ...
                0.5*(VBKFSEQ_Hk_d(2,:)*VBKFSEQ_Px_d*VBKFSEQ_Hk_d(2,:)'); % VB-M step done
%             VBKFSEQ_IGb1_d = tmp(1); VBKFSEQ_IGb2_d = tmp(2); % VB-M step done 
            if(norm(VBKFSEQ_X_d-VBKFSEQ_X_d_pre) < 1*VBKFSEQ_E_d)
                break;
            end
        end
        resVBKFSEQ_2nd_it_d(:,k) = i;
        resVBKFSEQ_it_d(:,k) = resVBKFSEQ_1st_it_d(:,k) + resVBKFSEQ_2nd_it_d(:,k);
        %res
        resVBKFSEQ_X_d(:,k) = VBKFSEQ_X_d;
        resVBKFSEQ_R_d(:,k) = [VBKFSEQ_R1_d;VBKFSEQ_R2_d];
        resVBKFSEQ_RMSE_d(:,k) = sqrt(trace(VBKFSEQ_Px_d));
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
            CRLB_D22 = inv(Q) + (H'/(TV_factor_of_R(k)*R))*H;
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
    % MC VBKF fix iteration result
    RMSE_VBKF_X_f(mc,1) = rmse(resVBKF_X_f(1,:),Xpse(1,:)); % state result
    RMSE_VBKF_X_f(mc,2) = rmse(resVBKF_X_f(2,:),Xpse(2,:));
    RMSE_VBKF_X_f(mc,3) = rmse(resVBKF_X_f(3,:),Xpse(3,:));
    RMSE_VBKF_X_f(mc,4) = rmse(resVBKF_X_f(4,:),Xpse(4,:)); 
    RMSE_VBKF_R_f(mc,1) = rmse(resVBKF_R_f(1,:),TV_factor_of_R.*R(1,1)); % measurement noise statistics result
    RMSE_VBKF_R_f(mc,2) = rmse(resVBKF_R_f(2,:),TV_factor_of_R.*R(2,2));
    % MC VBKF dynamic iteration result
    RMSE_VBKF_X_d(mc,1) = rmse(resVBKF_X_d(1,:),Xpse(1,:)); % state result
    RMSE_VBKF_X_d(mc,2) = rmse(resVBKF_X_d(2,:),Xpse(2,:));
    RMSE_VBKF_X_d(mc,3) = rmse(resVBKF_X_d(3,:),Xpse(3,:));
    RMSE_VBKF_X_d(mc,4) = rmse(resVBKF_X_d(4,:),Xpse(4,:)); 
    RMSE_VBKF_R_d(mc,1) = rmse(resVBKF_R_d(1,:),TV_factor_of_R.*R(1,1)); % measurement noise statistics result
    RMSE_VBKF_R_d(mc,2) = rmse(resVBKF_R_d(2,:),TV_factor_of_R.*R(2,2));    
    % MC VBKFSEQ dynamic iteration result
    RMSE_VBKFSEQ_X_d(mc,1) = rmse(resVBKFSEQ_X_d(1,:),Xpse(1,:)); % state result
    RMSE_VBKFSEQ_X_d(mc,2) = rmse(resVBKFSEQ_X_d(2,:),Xpse(2,:));
    RMSE_VBKFSEQ_X_d(mc,3) = rmse(resVBKFSEQ_X_d(3,:),Xpse(3,:));
    RMSE_VBKFSEQ_X_d(mc,4) = rmse(resVBKFSEQ_X_d(4,:),Xpse(4,:)); 
    RMSE_VBKFSEQ_R_d(mc,1) = rmse(resVBKFSEQ_R_d(1,:),TV_factor_of_R.*R(1,1)); % measurement noise statistics result
    RMSE_VBKFSEQ_R_d(mc,2) = rmse(resVBKFSEQ_R_d(2,:),TV_factor_of_R.*R(2,2));    
end
%% figures
% plot state
f1 = figure('Name','f1_State Estimation');
subplot(2,3,[1,2,4,5]);hold on;
plot(Xpse(1,:),Xpse(2,:));
plot(resKF_X(1,:),resKF_X(2,:));
plot(resVBKF_X_f(1,:),resVBKF_X_f(2,:));
plot(resVBKF_X_d(1,:),resVBKF_X_d(2,:));
plot(resVBKFSEQ_X_d(1,:),resVBKFSEQ_X_d(2,:));
plot(Ypse(1,:),Ypse(2,:),'*');
plot(X0(1),X0(2),'o');
legend('True','KF','VBKF_f','VBKF_d','VBKFSEQ_d','Observer');
set(ylabel(['$v$ direction position($m$)'],'Interpreter','latex'));
set(xlabel(['$u$ direction position($m$)'],'Interpreter','latex'));
subplot(2,3,3);hold on;
plot(Time,[Xpse(3,:);resKF_X(3,:);resVBKF_X_f(3,:);resVBKF_X_d(3,:);resVBKFSEQ_X_d(3,:)]');
set(ylabel(['$u$ direction velocity($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(2,3,6);hold on;
plot(Time,[Xpse(4,:);resKF_X(4,:);resVBKF_X_f(4,:);resVBKF_X_d(4,:);resVBKFSEQ_X_d(4,:)]');
set(ylabel(['$v$ direction velocity($m/s$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
sgtitle("State Estimation Result",'Fontsize',10,'Interpreter','Latex');
% plot measurement noise statistics
f2 = figure('Name','f2_Measurement Covariance Estimation');
subplot(211);
plot(Time,[TV_factor_of_R.*R(1,1).*ones(size(resVBKF_R_f(1,:)));resVBKF_R_f(1,:);...
    resVBKF_R_d(1,:);resVBKFSEQ_R_d(1,:)]');
set(ylabel(['$R_1$ ($m^2$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
subplot(212);
plot(Time,[TV_factor_of_R.*R(2,2).*ones(size(resVBKF_R_f(2,:)));resVBKF_R_f(2,:);...
    resVBKF_R_d(2,:);resVBKFSEQ_R_d(2,:)]');
legend('True','VBKF_f','VBKF_d','VBKFSEQ_d');
set(ylabel(['$R_2$ ($m^2$)'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
sgtitle("Measurement Covariance Estimation Result",'Fontsize',10,'Interpreter','Latex');
% plot iterated times
f3 = figure('Name','f3_Iterated Times');
plot(Time,[resVBKF_it_d;resVBKFSEQ_it_d;resVBKFSEQ_1st_it_d;resVBKFSEQ_2nd_it_d]');
legend('It. of VBKF_d','It. of VBKFSEQ_d','1st It. of VBKFSEQ_d','2nd It. of VBKFSEQ_d');
set(ylabel(['iterated times'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
title("Iterated Times",'Fontsize',10,'Interpreter','Latex');
% plot CRLB and RMSE
f4 = figure('Name','f4_CRLB and RMSE');
semilogy(Time,[resKF_RMSE;resVBKF_RMSE_f;resVBKF_RMSE_d;resVBKFSEQ_RMSE_d;resCRLB_trLB]');
legend('KF','VBKF_f','VBKF_d','VBKFSEQ_d','CRLB');
set(ylabel(['RMSE error'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
title("CRLB and RMSE",'Fontsize',10,'Interpreter','Latex');
% plot MC RMSE
if(MC_times > 1)
    C = lines(4);
    items={'KF','VBKF_f','VBKF_d','VBSEQ_d'};
    % state MC RMSE
    f5 = figure('Name','f5_state MC RMSE');
    subplot(221);boxplot([RMSE_KF_X(:,1),RMSE_VBKF_X_f(:,1),...
        RMSE_VBKF_X_d(:,1),RMSE_VBKFSEQ_X_d(:,1)],items,"Colors",C);
    set(ylabel(['Position $u$'],'Interpreter','latex'));
    subplot(222);boxplot([RMSE_KF_X(:,2),RMSE_VBKF_X_f(:,2),...
        RMSE_VBKF_X_d(:,2),RMSE_VBKFSEQ_X_d(:,2)],items,"Colors",C);
    set(ylabel(['Position $v$'],'Interpreter','latex'));
    subplot(223);boxplot([RMSE_KF_X(:,3),RMSE_VBKF_X_f(:,3),...
        RMSE_VBKF_X_d(:,3),RMSE_VBKFSEQ_X_d(:,3)],items,"Colors",C);
    set(ylabel(['Velocity $u$'],'Interpreter','latex'));
    subplot(224);boxplot([RMSE_KF_X(:,4),RMSE_VBKF_X_f(:,4),...
        RMSE_VBKF_X_d(:,4),RMSE_VBKFSEQ_X_d(:,4)],items,"Colors",C);
    set(ylabel(['Velocity $v$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo state RMSE Value",'Fontsize',10,'Interpreter','Latex');
    % measurement noise statistics MC RMSE
    f6 = figure('Name','f6_measurement noise statistics MC RMSE');
    subplot(211);boxplot([RMSE_VBKF_R_f(:,1), RMSE_VBKF_R_d(:,1),...
        RMSE_VBKFSEQ_R_d(:,1)],items(2:end),"Colors",C);
    set(ylabel(['$R_1$'],'Interpreter','latex'));
    subplot(212);boxplot([RMSE_VBKF_R_f(:,2), RMSE_VBKF_R_d(:,2),...
        RMSE_VBKFSEQ_R_d(:,2)],items(2:end),"Colors",C);
    set(ylabel(['$R_2$'],'Interpreter','latex'));
    sgtitle("Monte-Carlo measurement noise RMSE Value",'Fontsize',10,'Interpreter','Latex');
end
