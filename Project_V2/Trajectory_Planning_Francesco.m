clc
clear all
close all

run("ReLoadingScript.m");

Poses=[];
Euls=[];

for i=1:length(path.States)
    quat=[path.States(i,4) path.States(i,5) path.States(i,6) path.States(i,7)];
    eul=quat2eul(quat,'XYZ');
    Euls=[Euls;eul];
end

for i=1:length(path.States)
    Poses=[Poses; path.States(i,1) path.States(i,2) path.States(i,3) Euls(i,3)];
end

% for i=1:length(Poses)-1
%     Poses(i+1,4)=atan2(Poses(i+1,2),Poses(i+1,1));
% end

%% Time law calculation
% By considering ti=0 and tf chosen, imposng velocity and acceleration zero
% at starting and finishing points we have a0=a1=a2=0 and a set of three
% equations and 3 incognites. s(tf) is considered equal to 1. The found
% time law for position will be used for all the movement along axes and
% for the yaw. Similarly the found time law for velocity and acceleration.

%% INITIALIZATION

%% UAV PARAMETERS

mass = 1.2;
Ib = diag([1.2416 1.2416 2*1.2416]); 

%% SAMPLING TIME
Ts=0.001;

%% PLANNER
ti = 0;
t_f=20; %duration
tdead=5;
T=t_f+tdead;
t1=linspace(0,t_f,round(t_f/Ts));
ttot=T*(length(Poses)-1); %duration
t_d=linspace(0,tdead,round(tdead/Ts));
t=linspace(0,ttot,round(ttot/Ts));

p= []; 
dot_p= []; 
ddot_p= [];
p_temp= zeros(4,length(t1)); 
dot_p_temp= zeros(4,length(t1)); 
ddot_p_temp= zeros(4,length(t1));
p_d= zeros(4,length(t_f)); 
dot_p_d= zeros(4,length(t_f)); 
ddot_p_d= zeros(4,length(t_f));
a=ones(4,length(t_d));

%Initial and final conditions
for i=1:length(Poses)-1
x0=Poses(i,1);
y0=Poses(i,2);
z0=Poses(i,3);
psi0=Poses(i,4);

xf=Poses(i+1,1);
yf=Poses(i+1,2);
zf=Poses(i+1,3);
psif=Poses(i+1,4);

dot_x0= 0; dot_xf=0; ddot_x0=0; ddot_xf=0; dddot_x0 = 0; dddot_xf = 0;
dot_y0= 0; dot_yf=0; ddot_y0=0; ddot_yf=0; dddot_y0 = 0; dddot_yf = 0;
dot_z0= 0; dot_zf=0; ddot_z0=0; ddot_zf=0; dddot_z0 = 0; dddot_zf = 0;
dot_psi0= 0; dot_psif=0; ddot_psi0=0; ddot_psif=0; dddot_psi0 = 0; dddot_psif = 0;

p0=[x0 y0 z0 psi0]; dot_p0=[dot_x0,dot_y0,dot_z0,dot_psi0]; ddot_p0=[ddot_x0,ddot_y0,ddot_z0,ddot_psi0]; dddot_p0=[dddot_x0,dddot_y0,dddot_z0,dddot_psi0];
pf=[xf yf zf psif]; dot_pf=[dot_xf,dot_yf,dot_zf,dot_psif]; ddot_pf=[ddot_xf,ddot_yf,ddot_zf,ddot_psif]; dddot_pf=[dddot_xf,dddot_yf,dddot_zf,dddot_psif];
%7-th order polynomial
a0=zeros(1,4); a1=zeros(1,4); a2=zeros(1,4); a3=zeros(1,4); a4=zeros(1,4); a5=zeros(1,4); a6 = zeros(1,4); a7 = zeros(1,4);

for j=1:4
    A = [ti^7, ti^6, ti^5, ti^4, ti^3, ti^2, ti, 1;
        t_f^7, t_f^6, t_f^5, t_f^4, t_f^3, t_f^2, t_f, 1;
        7*ti^6, 6*ti^5, 5*ti^4, 4*ti^3, 3*ti^2, 2*ti, 1, 0;
        7*t_f^6, 6*t_f^5, 5*t_f^4, 4*t_f^3, 3*t_f^2, 2*t_f, 1, 0;
        42*ti^5, 30*ti^4, 20*ti^3, 12*ti^2, 6*ti, 2, 0, 0;
        42*t_f^5, 30*t_f^4, 20*t_f^3, 12*t_f^2, 6*t_f, 2, 0, 0;
        210*ti^4, 120*ti^3, 60*ti^2, 24*ti, 6, 0, 0, 0;
        210*t_f^4, 120*t_f^3, 60*t_f^2, 24*t_f, 6, 0, 0, 0];
    b = [p0(j) pf(j) dot_p0(j) dot_pf(j) ddot_p0(j) ddot_pf(j) dddot_p0(j) dddot_pf(j)]';
    a_temp = A\b;
    a7(j) = a_temp(1);
    a6(j) = a_temp(2);
    a5(j) = a_temp(3);
    a4(j) = a_temp(4);
    a3(j) = a_temp(5);
    a2(j) = a_temp(6);
    a1(j) = a_temp(7);
    a0(j) = a_temp(8);
    
    %trajectories
    p_temp(j,:)=a7(j)*t1.^7 + a6(j)*t1.^6 + a5(j)*t1.^5 +a4(j)*t1.^4 +a3(j)*t1.^3 +a2(j)*t1.^2 +a1(j)*t1 +a0(j);
    dot_p_temp(j,:) = 7*a7(j)*t1.^6 + 6*a6(j)*t1.^5 + 5*a5(j)*t1.^4 +4*a4(j)*t1.^3 +3*a3(j)*t1.^2 +2*a2(j)*t1 +a1(j);
    ddot_p_temp(j,:) = 42*a7(j)*t1.^5 + 30*a6(j)*t1.^4 + 5*4*a5(j)*t1.^3 +4*3*a4(j)*t1.^2 +3*2*a3(j)*t1 +2*a2(j);
end
p=[p p_temp];
dot_p=[dot_p dot_p_temp];
ddot_p=[ddot_p ddot_p_temp];

steady_p=a.*pf';
steady_dot_p=a.*dot_pf';
steady_ddot_p=a.*ddot_pf';

p=[p steady_p];
dot_p=[dot_p steady_dot_p];
ddot_p=[ddot_p steady_ddot_p];

end

figure("Name","Trajectory")
    hMap=show(Env3D)
    hold on
    scatter3(hMap,Start(1),Start(2),Start(3),30,"cyan","filled")
    scatter3(hMap,Goal(1),Goal(2),Goal(3),30,"green","filled")
    scatter3(waypoints(2:end-1,1),waypoints(2:end-1,2),waypoints(2:end-1,3),"y","filled")
    plot3(p(1,:),p(2,:),p(3,:),"magenta",LineWidth=2)
    hold off

%data for Simulink
pos_0 = [Poses(1,1) Poses(1,2) Poses(1,3)];
lin_vel_0 = [dot_x0 dot_y0 dot_z0];
csi_d= [p(1,:); p(2,:); p(3,:)]; dot_csi_d=[dot_p(1,:); dot_p(2,:); dot_p(3,:)]; ddot_csi_d=[ddot_p(1,:); ddot_p(2,:); ddot_p(3,:)];
psi_d= p(4,:); dot_psi_d=dot_p(4,:); ddot_psi_d=ddot_p(4,:);


%%%%%%%%%%%%%%%%%% Defining Transfer Function for 3-rd Order Estimator

K0=10;
s=tf('s');
G3=((K0)^3)/((s+K0)^3);
K3=flip(G3.Denominator{1});

