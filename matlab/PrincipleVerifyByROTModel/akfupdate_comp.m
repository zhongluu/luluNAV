function kf = akfupdate_comp(kf, yk, TimeMeasBoth, kftype, para)
% Adaptive Kalman filter. 
%
% Prototype: kf = akfupdate(kf, yk, TimeMeasBoth, kftype)
% Inputs: kf - Kalman filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
%        kftype - adaptive KF type
%        para - some parameters
% Output: kf - Kalman filter structure array after time/meas updating
%
% The MCKF/RSTKF/SSMKF/CERKF source code is provided by Lianzhao Wang & Fengchi Zhu.
%
% See also  kfinit, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/03/2022
lambda = 1;
SensorsPoint = [ 5e2,  5e2;
    -5e2,  5e2;
    5e2, -5e2;
    -5e2, -5e2];
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end
    
    if TimeMeasBoth=='T'            % Time Updating
%         kf.xk = kf.Phikk_1*kf.xk; 
        kf.Phikk_1 = JabF(kf.xk, 1);
        kf.xk = Ft(kf.xk, 1);
        kf.Pxk = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
    else
        if TimeMeasBoth=='M'        % Meas Updating
            kf.xkk_1 = kf.xk;
            kf.Pxkk_1 = kf.Pxk;
        elseif TimeMeasBoth=='B'    % Time & Meas Updating
%             kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Phikk_1 = JabF(kf.xk, 1);
            kf.xkk_1 = Ft(kf.xk, 1);
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        else
            error('TimeMeasBoth input error!');
        end
        kf.Hk = JabH(kf.xkk_1,SensorsPoint);
        kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';
        kf.Py0 = kf.Hk*kf.Pxykk_1;
%         kf.ykk_1 = kf.Hk*kf.xkk_1;
        kf.ykk_1 = Ht(kf.xkk_1,SensorsPoint);
        kf.rk = yk-kf.ykk_1;
        if ischar(kftype)
            switch(kftype)
                case 'KF',  kftype=0;  case 'MSHAKF',  kftype=1; b = para(:,1); if size(para,2)>1, s = para(:,2); else, s=[]; end
                case 'MCKF',  kftype=2; sigma=para(1); case 'RSTKF', kftype=3; v = para(1); case 'SSMKF',  kftype=4; w = para(1); 
                case 'CERKF', kftype=5; nu = para(1);
            end
        end
        
        lambda = 1;  maxflag = 0;
        xk_1 = kf.xk;
        for iter = 1:kf.measIter         
%             Bk = (yk-kf.Hk*kf.xk)*(yk-kf.Hk*kf.xk)' + kf.Hk*kf.Pxk*kf.Hk';
            Bk = (yk-Ht(kf.xk,SensorsPoint))*(yk-Ht(kf.xk,SensorsPoint))' + kf.Hk*kf.Pxk*kf.Hk';
            tbr = Bk*kf.Rk^(-1);
            switch(kftype)
                case 1,  % 改进Sage-Husa自适应卡尔曼滤波
                    r2 = kf.rk.^2 - diag(kf.Py0);
                    [kf.Rk, kf.betak, maxflag] = shamn(kf.Rk, r2, kf.Rmin, kf.Rmax, kf.betak, b, s);
                case 2,  % MCKF最大相关熵卡尔曼滤波
                    lambda = exp((1-trace(tbr))/2/sigma^2);
                case 3,  % RSTKF鲁棒学生t卡尔曼滤波
                    lambda = (v + kf.m)/(v + trace(tbr));
                case 4,  % SSMKF统计相似度量卡尔曼滤波
                    lambda = sqrt((w + kf.m)/(w + trace(tbr)));
                case 5,  % CERKF计算高效鲁棒卡尔曼滤波
                    lambda = (nu + kf.m)/(nu + kf.rk'*inv(kf.Py0 + kf.Rk)*kf.rk);
            end
            if lambda<1e-6, lambda = 1e-6;  end
            kf.Pykk_1 = kf.Py0 + kf.Rk/lambda;
            kf.Kk = kf.Pxykk_1*invbc(kf.Pykk_1);
            kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')*(1/2);
            kf.xk = kf.xkk_1 + kf.Kk*kf.rk;
            if kftype<=1 || kftype==5, break; end  % no need iteration
            if norm(kf.xk-xk_1)<norm(kf.xk)*1e-12, break; else, xk_1 = kf.xk; end
        end
    end
%     kf.res = sqrt(kf.Rk(1,1)/lambda);
    kf.res = (kf.Rk/lambda);
end

% system model
function Xnext = Ft(Xcur, deltaT)
Fk = [1, sin(Xcur(5)*deltaT)/Xcur(5),       0, (cos(Xcur(5)*deltaT) - 1)/Xcur(5), 0;
    0, cos(Xcur(5)*deltaT),               0, -sin(Xcur(5)*deltaT),              0;
    0, (1 - cos(Xcur(5)*deltaT))/Xcur(5), 1, sin(Xcur(5)*deltaT)/Xcur(5),       0;
    0, sin(Xcur(5)*deltaT),               0, cos(Xcur(5)*deltaT),               0;
    0, 0,                                 0, 0,                                 1;];

Xnext = Fk * Xcur;
end

function Ycur = Ht(Xcur, Sensors)
[r, ~] = size(Sensors);
Ycur = zeros(r,1);
for i = 1 : r
    Ycur(i,:) = sqrt((Xcur(1) - Sensors(i, 1))^2 + (Xcur(3) - Sensors(i, 2))^2);
end
end

function Fk = JabF(Xcur, deltaT)
Fk = [1, sin(Xcur(5)*deltaT)/Xcur(5),       0, (cos(Xcur(5)*deltaT) - 1)/Xcur(5), (((deltaT*Xcur(5)*cos(Xcur(5)*deltaT)-sin(Xcur(5)*deltaT))*Xcur(2))-((deltaT*Xcur(5)*sin(Xcur(5)*deltaT)+cos(Xcur(5)*deltaT)-1)*Xcur(4)))/Xcur(5)^2;
    0, cos(Xcur(5)*deltaT),               0, -sin(Xcur(5)*deltaT),              -deltaT*(sin(Xcur(5)*deltaT)*Xcur(2)+ cos(Xcur(5)*deltaT)*Xcur(4));
    0, (1 - cos(Xcur(5)*deltaT))/Xcur(5), 1, sin(Xcur(5)*deltaT)/Xcur(5),       (((deltaT*sin(Xcur(5)*deltaT)*Xcur(5)-1+cos(Xcur(5)*deltaT)))*Xcur(2)+(deltaT*cos(Xcur(5)*deltaT))*Xcur(5)-sin(Xcur(5)*deltaT)*Xcur(4))/Xcur(5)^2;
    0, sin(Xcur(5)*deltaT),               0, cos(Xcur(5)*deltaT),               deltaT*(cos(Xcur(5)*deltaT)*Xcur(2)-sin(Xcur(5)*deltaT)*Xcur(4));
    0, 0,                                 0, 0,                                 1;];
end

function Hk = JabH(Xcur, Sensors)
[r, ~] = size(Sensors);
Hk = zeros(r, length(Xcur));
for i = 1 : r
    Hk(i,:) = [2*(Xcur(1) - Sensors(i,1))/sqrt((Xcur(1) - Sensors(i,1))^2 + (Xcur(3) - Sensors(i,2))^2), 0, 2*(Xcur(3) - Sensors(i,1))/sqrt((Xcur(1) - Sensors(i,1))^2 + (Xcur(3) - Sensors(i,2))^2), 0, 0];
end
end
    