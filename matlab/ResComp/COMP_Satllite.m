clear;
close all;
ggpsvars
load("../EXNAVDATA_1031_1807_I.mat");
load("../EXNAVDATA_1031_1807_I_SPP.mat");
BIGVB = load("VB21seq_1807_I.mat");
Chi = load("TC21CHI_1807_I.mat");

tend = length(posL);

obsPos = zeros(tend,3);
obsXYZ = zeros(tend,3);

obsPos(:,1) = deg2rad(posL(:,1));
obsPos(:,2) = deg2rad(posL(:,2));
obsPos(:,3) = posL(:,3);

map = containers.Map('KeyType','double','ValueType','any');
t=1;
for i = 1:tend
    num = GNSS.no_GNSS(i,1);
    curGNSS = GNSS.obs(:,:, i);
    staDly = curGNSS(1:num,5);
    lonoDly = curGNSS(1:num,6);
    tropDly = curGNSS(1:num,7);
    prn = GNSS.prn(i,:);
    for j =1:num
        if isKey(map,prn(j))
            curVBMInfo = map(prn(j));
        else
            curVBMInfo.SatAndPRt = [];
            curVBMInfo.time = [];
        end
        sat_r_es_e = curGNSS(j, 2:4);
        rV = curGNSS(j,1)+staDly(j) +lonoDly(j) +  tropDly(j);
        curVBMInfo.SatAndPRt = [curVBMInfo.SatAndPRt;sat_r_es_e, rV];
        curVBMInfo.time = [curVBMInfo.time, t];
        map(prn(j)) = curVBMInfo;
    end
    t=t+1;
end
keylist = keys(map);
valueList = values(map);
[~,n] = size(valueList);
SatUsedt = zeros(n,4,tend);
for i =1:n
    curVBMInfo = valueList{i};
    m = length(curVBMInfo.time);
    for j = 1:m
        SatUsedt(i,:,curVBMInfo.time(j)) = curVBMInfo.SatAndPRt(j,:);
    end
end


SatNums = size(BIGVB.Rt,1);
for i = 2:tend
    obsXYZ(i,:) = blh2xyz(obsPos(i,:));
end
PRN = 0:SatNums-1;

SatPrn = zeros(3,SatNums);

for i = 1:SatNums
    key = keylist{1,i};
    realPrn = floor(key/256);
    realFreqType = floor((key - realPrn*256)/8); 
    realType = key - (realPrn*256) - (realFreqType*8);
    SatPrn(1,i) = realPrn; SatPrn(2,i) = realType; SatPrn(3,i) = realFreqType; 
end

% satsECEF = [double(gnssRAW.satECEFx(1:len)),double(gnssRAW.satECEFy(1:len)),double(gnssRAW.satECEFz(1:len))];
NormalElevaAng = zeros(tend,SatNums);
NormalAzimuAng = zeros(tend,SatNums);
VBElevaAng = zeros(tend,SatNums);
VBAzimuAng = zeros(tend,SatNums);
ChiElevaAng = zeros(tend,SatNums);
ChiAzimuAng = zeros(tend,SatNums);

NormalEffsatsnum = zeros(tend);
NormalEffsats = zeros(size(BIGVB.SatUsedt));
VBEffsatsnum = zeros(tend);
VBEffsats = zeros(size(BIGVB.SatUsedt));
ChiEffsatsnum = zeros(tend);
ChiEffsats = zeros(size(Chi.SatUsedt));
for j = 1:tend
    NormalECEF = SatUsedt(:,1:3,j);
    VBsatsECEF = BIGVB.SatUsedt(:,1:3,j);
    ChisatsECEF = Chi.SatUsedt(:,1:3,j);

    B = obsPos(j,1); L = obsPos(j,2);
    R = [-sin(L),           cos(L), 0;
        -sin(B)*cos(L),  -sin(B)*sin(L), cos(B) ;
        cos(B)*cos(L),   cos(B)*sin(L), sin(B)];
    r = zeros(3,SatNums); kVB =1; kChi = 1;kNormal = 1;
 
 
    for i = 1:SatNums

           %% Normal
        if(NormalECEF(i,1)==0)
            Eleva = 0;
            Azimu = 0;
        else
            NormalEffsats(kNormal,:,j) = SatUsedt(i,:,j);
            kNormal = kNormal + 1;
            r(:,i) = NormalECEF(i,:)' - obsXYZ(j,:)';
            tmp = R *  r(:,i);
            E =tmp(1); N = tmp(2); U = tmp(3);
            elevation_angle = atan2(U,sqrt(E^2+N^2));
            Eleva =rad2deg( elevation_angle);
            azimuth = atan2(E,N);
            Azimu = mod(rad2deg(azimuth), 360);
        end

        NormalElevaAng(j,i) = Eleva;
        NormalAzimuAng(j,i) = Azimu;
           %% VB
        if(VBsatsECEF(i,1)==0)
            Eleva = 0;
            Azimu = 0;
        else
            VBEffsats(kVB,:,j) = BIGVB.SatUsedt(i,:,j);
            kVB = kVB + 1;
            r(:,i) = VBsatsECEF(i,:)' - obsXYZ(j,:)';
            tmp = R *  r(:,i);
            E =tmp(1); N = tmp(2); U = tmp(3);
            elevation_angle = atan2(U,sqrt(E^2+N^2));
            Eleva =rad2deg( elevation_angle);
            azimuth = atan2(E,N);
            Azimu = mod(rad2deg(azimuth), 360);
        end

        VBElevaAng(j,i) = Eleva;
        VBAzimuAng(j,i) = Azimu;
        %% chi
        if(ChisatsECEF(i,1)==0)
            Eleva = 0;
            Azimu = 0;
        else
            ChiEffsats(kChi, :, j) = Chi.SatUsedt(i,:,j);
            kChi = kChi + 1;
            r(:,i) = ChisatsECEF(i,:)' - obsXYZ(j,:)';
            tmp = R *  r(:,i);
            E =tmp(1); N = tmp(2); U = tmp(3);
            elevation_angle = atan2(U,sqrt(E^2+N^2));
            Eleva =rad2deg( elevation_angle);
            azimuth = atan2(E,N);
            Azimu = mod(rad2deg(azimuth), 360);
        end
        ChiElevaAng(j,i) = Eleva;
        ChiAzimuAng(j,i) = Azimu;
    end
    NormalEffsatsnum(j) = kVB -1;
    VBEffsatsnum(j) = kVB -1;
    ChiEffsatsnum(j) = kChi -1;

end

VBdopres = zeros(5,tend);Chidopres= zeros(5,tend);Normdopres = zeros(5,tend);

for i = 1:tend
    tgps = i;
    %% norm
    recPos= zeros(4,1);
    nums = NormalEffsatsnum(i);
    satpv = NormalEffsats(1:nums,1:3,tgps);
    
    CA = NormalEffsats(1:nums,4,tgps);

    [pvti, ~, ~] = lspvt2(recPos, satpv, CA);

    Normdopres(:,i) = pvti(7:11);
     %% VB
    recPos= zeros(4,1);
    nums = VBEffsatsnum(i);
    satpv = VBEffsats(1:nums,1:3,tgps);
    
    CA = VBEffsats(1:nums,4,tgps);

    [pvti, ~, ~] = lspvt2(recPos, satpv, CA);

    VBdopres(:,i) = pvti(7:11);
    %% chi
    recPos= zeros(4,1);
    nums = ChiEffsatsnum(i);
    satpv = ChiEffsats(1:nums,1:3,tgps);
    
    CA = ChiEffsats(1:nums,4,tgps);

    [pvti, ~, ~] = lspvt2(recPos, satpv, CA);

    Chidopres(:,i) = pvti(7:11);

end
NormalElevaAng(NormalElevaAng==0) = missing;
NormalAzimuAng(NormalAzimuAng==0) = missing;
VBElevaAng(VBElevaAng==0) = missing;
VBAzimuAng(VBAzimuAng==0) = missing;
ChiElevaAng(ChiElevaAng==0) = missing;
ChiAzimuAng(ChiAzimuAng==0) = missing;
NormalAzimuAngEnd = zeros(1,SatNums);
NormalElevaAngEnd = zeros(1,SatNums);
ChiAzimuAngEnd = zeros(1,SatNums);
ChiElevaAngEnd = zeros(1,SatNums);
VBAzimuAngEnd = zeros(1,SatNums);
VBElevaAngEnd = zeros(1,SatNums);
GNSS_SYS = {'GPS','GLONASS','SBAS','GAL','BDS','QZSS','IRNSS'};
leg = cell(1,SatNums);
for i = 1:SatNums
%     leg{i} = strcat(GNSS_SYS(SatPrn(2,i)+1),':',num2str(SatPrn(1,i)));
    leg{i} = num2str(SatPrn(1,i));
    tmpA = NormalAzimuAng(~isnan(NormalAzimuAng(:,i)),i);
    tmpE = NormalElevaAng(~isnan(NormalElevaAng(:,i)),i);
    if size(tmpA,1) == 0
        NormalAzimuAngEnd(i) = NaN;NormalElevaAngEnd(i) = NaN;
    else
        NormalAzimuAngEnd(i) = tmpA(end);NormalElevaAngEnd(i) = tmpE(end);
    end
    tmpA = ChiAzimuAng(~isnan(ChiAzimuAng(:,i)),i);
    tmpE = ChiElevaAng(~isnan(ChiElevaAng(:,i)),i);
    if size(tmpA,1) == 0
        ChiAzimuAngEnd(i) = NaN;ChiElevaAngEnd(i) = NaN;
    else
        ChiAzimuAngEnd(i) = tmpA(end);ChiElevaAngEnd(i) = tmpE(end);
    end
    tmpA = VBAzimuAng(~isnan(VBAzimuAng(:,i)),i);
    tmpE = VBElevaAng(~isnan(VBElevaAng(:,i)),i);
    if size(tmpA,1) == 0
        VBAzimuAngEnd(i) = NaN;VBElevaAngEnd(i) = NaN;
    else
        VBAzimuAngEnd(i) = tmpA(end);VBElevaAngEnd(i) = tmpE(end);
    end
    
end
c =  lines(SatNums);

f1 = figure;
f1.Units = "inches";
f1.Position = [5.833333333333333,2.979166666666667,11.6667,4.375];
subplot(131);
for i = 1:SatNums
    color = c(SatPrn(2,i)+1,:);
    polarplot(deg2rad(NormalAzimuAng(:,i)),deg2rad(NormalElevaAng(:,i)),LineWidth=1,Color=color); hold on;
    polarplot(deg2rad(NormalAzimuAngEnd(i)),deg2rad(NormalElevaAngEnd(i)),...
        MarkerFaceColor=color,Marker='o',MarkerEdgeColor=color);
    text(deg2rad(NormalAzimuAngEnd(1,i)),deg2rad(NormalElevaAngEnd(1,i)),leg{1,i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment','bottom',...
        'FontName','Times New Roman','FontSize',8,'Color',color);
end
ax=gca; ax.Position = [0.05,0.11,0.25,0.815];
ax.ThetaZeroLocation = 'top';
ax.RTick=deg2rad([0,5,15,30,45,60,75,90]);
ax.RTickLabel={'','5','15','30','45','60','75',''};
ax.RDir='reverse';
ax.ThetaTickLabel={'North','30^{o}','60^{o}','East','120^{o}','150^{o}',...
    'South','210^{o}','240^{o}','West','300^{o}','330^{o}'};
set(gca,'FontName','Times New Roman','ThetaDir','clockwise',...
    'GridLineStyle','--','gridalpha',0.5,'Fontsize',11);
set(title(['Normal'],'Interpreter','latex'));
subplot(132);
for i = 1:SatNums
    color = c(SatPrn(2,i)+1,:);
    polarplot(deg2rad(ChiAzimuAng(:,i)),deg2rad(ChiElevaAng(:,i)),LineWidth=1,Color=color); hold on;
    polarplot(deg2rad(ChiAzimuAngEnd(i)),deg2rad(ChiElevaAngEnd(i)),...
        MarkerFaceColor=color,Marker='o',MarkerEdgeColor=color);
    text(deg2rad(ChiAzimuAngEnd(1,i)),deg2rad(ChiElevaAngEnd(1,i)),leg{1,i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment','bottom',...
        'FontName','Times New Roman','FontSize',8,'Color',color);
end
ax=gca; ax.Position = [0.378,0.11,0.25,0.815];
ax.ThetaZeroLocation = 'top';
ax.RTick=deg2rad([0,5,15,30,45,60,75,90]);
ax.RTickLabel={'','5','15','30','45','60','75',''};
ax.RDir='reverse';
ax.ThetaTickLabel={'North','30^{o}','60^{o}','East','120^{o}','150^{o}',...
    'South','210^{o}','240^{o}','West','300^{o}','330^{o}'};
set(gca,'FontName','Times New Roman','ThetaDir','clockwise',...
    'GridLineStyle','--','gridalpha',0.5,'Fontsize',11);
set(title(['TC-$\chi^2$'],'Interpreter','latex'));
subplot(133);
for i = 1:SatNums
    color = c(SatPrn(2,i)+1,:);
    polarplot(deg2rad(VBAzimuAng(:,i)),deg2rad(VBElevaAng(:,i)),LineWidth=1,Color=color); hold on;
    polarplot(deg2rad(VBAzimuAngEnd(i)),deg2rad(VBElevaAngEnd(i)),...
        MarkerFaceColor=color,Marker='o',MarkerEdgeColor=color);
    text(deg2rad(VBAzimuAngEnd(1,i)),deg2rad(VBElevaAngEnd(1,i)),leg{1,i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment','bottom',...
        'FontName','Times New Roman','FontSize',8,'Color',color);
end
ax=gca; ax.Position = [0.71,0.11,0.25,0.815];
ax.ThetaZeroLocation = 'top';
ax.RTick=deg2rad([0,5,15,30,45,60,75,90]);
ax.RTickLabel={'','5','15','30','45','60','75',''};
ax.RDir='reverse';
ax.ThetaTickLabel={'North','30^{o}','60^{o}','East','120^{o}','150^{o}',...
    'South','210^{o}','240^{o}','West','300^{o}','330^{o}'};
set(gca,'FontName','Times New Roman','ThetaDir','clockwise',...
    'GridLineStyle','--','gridalpha',0.5,'Fontsize',11);
set(title(['Proposed Method'],'Interpreter','latex'));
pos = get(f1,'Position');
set(f1,'Units','Inches');
set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


f2 = figure;
subplot(411);plot([Normdopres(1,:);Chidopres(1,:);VBdopres(1,:)]','LineWidth',2);
set(ylabel(['GDOP'],'Interpreter','latex'));
ax = gca; ax.XGrid = "on"; ax.XMinorGrid = "on"; ax.YGrid = "on";
ax.YMinorGrid="on"; ax.YLim = [1,3.5];
subplot(412);plot([Normdopres(2,:);Chidopres(2,:);VBdopres(2,:)]','LineWidth',2);
ax = gca; ax.XGrid = "on"; ax.XMinorGrid = "on"; ax.YGrid = "on";
ax.YMinorGrid="on"; ax.YLim = [1,2];
set(ylabel(['PDOP'],'Interpreter','latex'));
subplot(413);plot([Normdopres(3,:);Chidopres(3,:);VBdopres(3,:)]','LineWidth',2);
ax = gca; ax.XGrid = "on"; ax.XMinorGrid = "on"; ax.YGrid = "on";
ax.YMinorGrid="on"; ax.YLim = [0.6,1.6];
set(ylabel(['HDOP'],'Interpreter','latex'));
subplot(414);plot([Normdopres(4,:);Chidopres(4,:);VBdopres(4,:)]','LineWidth',2);
ax = gca; ax.XGrid = "on"; ax.XMinorGrid = "on"; ax.YGrid = "on";
ax.YMinorGrid="on"; ax.YLim = [0.5,1];
set(ylabel(['VDOP'],'Interpreter','latex'));
set(xlabel(['Time($s$)'],'Interpreter','latex'));
legend('Norm','TC-EKFwithChi','VB');
sgtitle("DOP",'Fontsize',10,'Interpreter','Latex');
pos = get(f2,'Position');
set(f2,'Units','Inches');
set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);


