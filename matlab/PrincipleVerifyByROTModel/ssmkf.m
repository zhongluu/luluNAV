function [xkk,Pkk,lbound,smfFinal]=ssmkf(xkk,Pkk,F,H,z,Q,R,sigma,v,N,epsilon,flag)

%%%%%%׼��
nz=size(z,1);

nx=size(xkk,1);

%%%%%ʱ�����
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%�������
xkk=xk1k;

Pkk=Pk1k;

lamda1=1;

lamda2=1;

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;

    %%%%%%%����״̬����
    D_Pk1k=Pk1k/lamda1;

    D_R=R/lamda2;

    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%%�ж�
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=epsilon
        break;
    end

    %%%%%%%���㸨������
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';

    %%%%%%%���㸨������
    gama1=trace(Dk1*inv(Pk1k));
    gama2=trace(Dk2*inv(R));
    
    %%%%%%%
%     [lamda1,lamda2]=fun_ap(gama1,gama2,sigma,v,flag,nx,nz);
    [~,lamda2]=fun_ap(gama1,gama2,sigma,v,flag,nx,nz);
end
