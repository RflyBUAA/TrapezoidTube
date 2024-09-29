clear;clc;clf
global  N UAV_Initial
N=120;
ts=0.001;

UAV_PInit=zeros(N,2);
UAV_PInit(1,:)=[0;0];UAV_PInit(2,:)=[0;1];UAV_PInit(3,:)=[0;2];UAV_PInit(4,:)=[0;3];UAV_PInit(5,:)=[0;4];
UAV_PInit(6,:)=[1;0];UAV_PInit(7,:)=[1;1];UAV_PInit(8,:)=[1;2];UAV_PInit(9,:)=[1;3];UAV_PInit(10,:)=[1;4];
UAV_PInit(11,:)=[2;0];UAV_PInit(12,:)=[2;1];UAV_PInit(13,:)=[2;2];UAV_PInit(14,:)=[2;3];UAV_PInit(15,:)=[2;4];
UAV_PInit(16,:)=[3;0];UAV_PInit(17,:)=[3;1];UAV_PInit(18,:)=[3;2];UAV_PInit(19,:)=[3;3];UAV_PInit(20,:)=[3;4];

for i=1:20
    UAV_PInit(i,:)=UAV_PInit(i,:)+[0,3];
end
for i=21:40
    UAV_PInit(i,:)=UAV_PInit(i-20,:)+[-4,0];
end
for i=41:80
    UAV_PInit(i,:)=UAV_PInit(i-40,:)+[0,-5];
end
for i=81:120
    UAV_PInit(i,:)=UAV_PInit(i-80,:)+[0,-10];
end
UAV_Initial=[UAV_PInit(:,1);UAV_PInit(:,2)];

vmax=3.5;
vmin=0.5;
vline=2;
vmax1=1.5;
k1=1;

rs=0.25;
ra=0.5;

p_leader(1,:)=[-5,0,0];
p_leader(2,:)=[35,0,0];
dis=[10 4];

p_left=[];
p_right=[];
for i=1:length(p_leader(:,1))
    p_left(i,:)=(p_leader(i,1:2))+dis(i)*[cos(p_leader(i,3)+pi/2),sin(p_leader(i,3)+pi/2)];
    p_right(i,:)=(p_leader(i,1:2))+dis(i)*[cos(p_leader(i,3)-pi/2),sin(p_leader(i,3)-pi/2)];
end

p_obstacle=[6,-4;13.5,3;20,-2;27,2;33,0];
ro=[0.9;0.9;0.9;0.9;0.9];
so=[3*ro(1);5*ro(2);4*ro(3);5*ro(4);3*ro(5)];

%%
tunnel_left=p_left;
tunnel_right=p_right;

ne=zeros(length(tunnel_left(:,1))-1,2);
nl=zeros(length(tunnel_left(:,1))-1,2);
nr=zeros(length(tunnel_left(:,1))-1,2);
ns=zeros(length(tunnel_left(:,1))-1,2);

c=zeros(length(tunnel_left(:,1))-1,2);
l=zeros(length(tunnel_left(:,1))-1,2);
r=zeros(length(tunnel_left(:,1))-1,2);
s=zeros(length(tunnel_left(:,1))-1,2);

p_fl=zeros(length(tunnel_left(:,1))-1,2);
p_fr=zeros(length(tunnel_left(:,1))-1,2);
p_sl=zeros(length(tunnel_left(:,1))-1,2);
p_sr=zeros(length(tunnel_left(:,1))-1,2);

line_left=diff(tunnel_left);
line_right=diff(tunnel_right);
line_end=tunnel_right(2:end,:)-tunnel_left(2:end,:);
line_start=tunnel_right(1:end-1,:)-tunnel_left(1:end-1,:);

for i=1:length(tunnel_left(:,1))-1
    rot_ne=[0 1;-1 0]; 
    rot_nl=[0 1;-1 0];
    rot_nr=[0 -1;1 0];
    rot_ns=[0 -1;1 0];

    A=rot_ne*line_end(i,:)';
    B=rot_nl*line_left(i,:)';
    C=rot_nr*line_right(i,:)';
    D=rot_ns*line_start(i,:)';

    ne(i,:)=(A/norm(A))';
    nl(i,:)=(B/norm(B))';
    nr(i,:)=(C/norm(C))';
    ns(i,:)=(D/norm(D))';

    c(i,:)=line_end(i,:)/norm(line_end(i,:));
    l(i,:)=line_left(i,:)/norm(line_left(i,:));
    r(i,:)=line_right(i,:)/norm(line_right(i,:));
    s(i,:)=line_start(i,:)/norm(line_start(i,:));

end

for j=1:length(tunnel_left(:,1))-1
    p_fl(j,:)=tunnel_left(j+1,:);
    p_fr(j,:)=tunnel_right(j+1,:);
    p_sl(j,:)=tunnel_left(j,:);
    p_sr(j,:)=tunnel_right(j,:);
end

obstalce_locate=zeros(length(p_obstacle(:,1)),1);
for i=1:length(p_obstacle(:,1))
    for j=1:length(tunnel_left(:,1))-1
        if dot(nl(j,:),(p_obstacle(i,:)-p_fl(j,:)))>=0 && dot(nr(j,:),(p_obstacle(i,:)-p_fr(j,:)))>=0 ...
                && dot(ne(j,:),(p_obstacle(i,:)-p_fr(j,:)))>=0 && dot(ns(j,:),(p_obstacle(i,:)-p_sr(j,:)))>=0
            obstalce_locate(i)=j;
        end
    end
end
%%
potl=zeros(length(p_obstacle(:,1)),2);
potu=zeros(length(p_obstacle(:,1)),2);
potd=zeros(length(p_obstacle(:,1)),2);
pol0=zeros(length(p_obstacle(:,1)),2);
pol1=zeros(length(p_obstacle(:,1)),2);
por0=zeros(length(p_obstacle(:,1)),2);
por1=zeros(length(p_obstacle(:,1)),2);
for i=1:length(p_obstacle(:,1))
    Aol=zeros(2,2);Aor=zeros(2,2);
    potl(i,:)=p_obstacle(i,:)+so(i)*ne(obstalce_locate(i),:);
    potu(i,:)=p_obstacle(i,:)-ro(i)*ne(obstalce_locate(i),:)-(ro(i)^2+so(i)*ro(i))/(sqrt(so(i)^2-ro(i)^2))*c(obstalce_locate(i),:);
    potd(i,:)=p_obstacle(i,:)-ro(i)*ne(obstalce_locate(i),:)+(ro(i)^2+so(i)*ro(i))/(sqrt(so(i)^2-ro(i)^2))*c(obstalce_locate(i),:);

    Aol=[-l(obstalce_locate(i),2)*c(obstalce_locate(i),1),l(obstalce_locate(i),1)*c(obstalce_locate(i),1);-l(obstalce_locate(i),2)*c(obstalce_locate(i),2),l(obstalce_locate(i),1)*c(obstalce_locate(i),2)]/(l(obstalce_locate(i),1)*c(obstalce_locate(i),2)-l(obstalce_locate(i),2)*c(obstalce_locate(i),1));
    Aor=[-r(obstalce_locate(i),2)*c(obstalce_locate(i),1),r(obstalce_locate(i),1)*c(obstalce_locate(i),1);-r(obstalce_locate(i),2)*c(obstalce_locate(i),2),r(obstalce_locate(i),1)*c(obstalce_locate(i),2)]/(r(obstalce_locate(i),1)*c(obstalce_locate(i),2)-r(obstalce_locate(i),2)*c(obstalce_locate(i),1));
    pol0(i,:)=potl(i,:)+(Aol*(p_fl(obstalce_locate(i),:)-potl(i,:))')';
    pol1(i,:)=potu(i,:)+(Aol*(p_fl(obstalce_locate(i),:)-potu(i,:))')';
    por0(i,:)=potl(i,:)+(Aor*(p_fr(obstalce_locate(i),:)-potl(i,:))')';
    por1(i,:)=potd(i,:)+(Aor*(p_fr(obstalce_locate(i),:)-potd(i,:))')';
end

sim("TrapezoidSimulation")
DrawFinal




