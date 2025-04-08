function [xkk,Pkk,xkk_A,Pkk_A,ykk,Ykk,ukk,Ukk,Q,R,Pk1k]=new_aprivbkf_comp(xkk,Pkk,xkk_A,Pkk_A,ykk,Ykk,ukk,Ukk,zA,Q,R,rou,L,t)

%%%%%
nx=size(xkk,1);
SensorsPoint = [ 5e2,  5e2;
                -5e2,  5e2;
                 5e2, -5e2;
                -5e2, -5e2];
%%%%%%%%%%%%%%%%%%%%%%%%%Forward filtering
%%%%%Time update
% xk1k=F*xkk;
% 
% Pk1k=F*Pkk*F'+Q; 
F = JabF(xkk, 1);
xk1k=Ft(xkk,1);
Pk1k=F*Pkk*F'+Q; 

%%%%%Measurement update
z=zA(:,end);

% zk1k=H*xk1k;
zk1k= Ht(xk1k,SensorsPoint);
H = JabH(xk1k, SensorsPoint);
Pzzk1k=H*Pk1k*H'+R;

Pxzk1k=Pk1k*H';

Kk=Pxzk1k*inv(Pzzk1k);

xkk=xk1k+Kk*(z-zk1k);

Pkk=Pk1k-Kk*H*Pk1k;

%%%%%%%%%%%%%%%%%%%%%%%%%
if t<=(L+1)
    xkk_A=[xkk_A xkk];
    Pkk_A=[Pkk_A Pkk];
else
    xkk_A=[xkk_A(:,2:end) xkk];
    Pkk_A=[Pkk_A(:,(nx+1):end) Pkk];
end

%%%%%%%%%%%%%%%%%%%%%%%%%Backward smoothing
if t>=2
    %%%%%%Prior information
    yk1k=rou*ykk;
    Yk1k=rou*Ykk;
    uk1k=rou*ukk;
    Uk1k=rou*Ukk;
    xkN=xkk;
    PkN=Pkk;
    
    %%%%%%
    Ak=zeros(nx);
%     Bk=(z-H*xkN)*(z-H*xkN)'+H*PkN*H'; 
    Bk=(z-Ht(xkN,SensorsPoint))*(z-Ht(xkN,SensorsPoint))'+H*PkN*H'; 
    %%%%%%
    if t<=(L+1)
        M=t-1;
    else
        M=L;
    end
    
    for j=M:-1:1
        %%%%%%
        xk_1=xkk_A(:,j);
        Pk_1=Pkk_A(:,(j-1)*nx+1:j*nx);
        
        %%%%%%
        F = JabF(xk_1,1);
        xkk_1=Ft(xk_1,1);
%         xkk_1=F*xk_1;
        Pkk_1=F*Pk_1*F'+Q;
        Gk_1=Pk_1*F'*inv(Pkk_1);
        xk_1N=xk_1+Gk_1*(xkN-xkk_1);
        Pk_1N=Pk_1-Gk_1*(Pkk_1-PkN)*Gk_1';
        
        %%%%%%
        Pk_1kN=Gk_1*PkN;
        F_Q=PkN-(F*Pk_1kN)'-F*Pk_1kN+F*Pk_1N*F'+(xkN-F*xk_1N)*(xkN-F*xk_1N)';
        Ak=Ak+F_Q;
        
        %%%%%%
        z=zA(:,j);
%         F_R=(z-H*xk_1N)*(z-H*xk_1N)'+H*Pk_1N*H'; 
        F_R=(z-Ht(xk_1N,SensorsPoint))*(z-Ht(xk_1N,SensorsPoint))'+H*Pk_1N*H'; 
        Bk=Bk+F_R;
        
        %%%%%%
        xkN=xk_1N;
        PkN=Pk_1N;
        
    end
    
    %%%%%%Update distribution parameters of Q
    ykk=yk1k+M;
    Ykk=Yk1k+Ak;

    %%%%%%Update estimate of Q
    Q=Ykk/ykk;
    
    %%%%%%Update distribution parameters of R
    ukk=uk1k+M+1;
    Ukk=Uk1k+Bk;
    
    %%%%%%Update estimate of R
    R=Ukk/ukk;
    
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
