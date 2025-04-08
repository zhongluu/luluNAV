function [xkk,Pkk,D_R,lbound,smfFinal]=ssmkf_comp(xkk,Pkk,z,Q,R,sigma,v,N,epsilon,flag)

%%%%%%准备
nz=size(z,1);

nx=size(xkk,1);
SensorsPoint = [ 5e2,  5e2;
    -5e2,  5e2;
    5e2, -5e2;
    -5e2, -5e2];
%%%%%时间更新
% xk1k=F*xkk;
F = JabF(xkk, 1);
xk1k=Ft(xkk,1);
Pk1k=F*Pkk*F'+Q;

%%%%%量测更新
xkk=xk1k;

Pkk=Pk1k;

lamda1=1;

lamda2=1;

for i=1:N

    %%%%%%%
    xkk_i=xkk;

    %%%%%%%计算状态估计
    D_Pk1k=Pk1k/lamda1;

    D_R=R/lamda2;

    %     zk1k=H*xk1k;
    zk1k= Ht(xk1k,SensorsPoint);
    H = JabH(xk1k, SensorsPoint);
    Pzzk1k=H*D_Pk1k*H'+D_R;

    Pxzk1k=D_Pk1k*H';

    Kk=Pxzk1k*inv(Pzzk1k);

    xkk=xk1k+Kk*(z-zk1k);

    Pkk=D_Pk1k-Kk*H*D_Pk1k;

    %%%%%%%%判定
    td=norm(xkk-xkk_i)/norm(xkk);

    if td<=epsilon
        break;
    end

    %%%%%%%计算辅助参数
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;

%     Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
     Dk2=(z-Ht(xkk,SensorsPoint))*(z-Ht(xkk,SensorsPoint))'+H*Pkk*H';
    %%%%%%%计算辅助参数
    gama1=trace(Dk1*inv(Pk1k));
    gama2=trace(Dk2*inv(R));

    %%%%%%%
    [lamda1,lamda2]=fun_ap(gama1,gama2,sigma,v,flag,nx,nz);

end
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
