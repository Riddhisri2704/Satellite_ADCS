addpath 'C:\Users\Lenovo\MATLAB\Projects\ADCS'

clear
clc
close all

tic

global BB lastMagUpdate nextMagUpdate lastSensorUpdate nextSensorUpdate BfieldMeasured pqrMeasured 
global BfieldNav pqrNav BfieldNavPrev pqrNavPrev

BfieldNavPrev= [0;0;0];
pqrNavPrev= [0;0;0];

%setup IGRF model               
%igrf works only uptill 1 jan 2020
addpath 'C:\Users\Lenovo\Documents\ADCS\igrf'


%%%get planet parameters
planet

%get mass and inertia properties
inertia

%initial conditions for position and velocity
altitude= 600*1000;                         %%%meters
x0= R+ altitude;
y0=0;
z0=0;
xdot0=0;                                    %%%velocity normal to earth surface
semi_major=norm([x0;y0;z0]);
vcircular= sqrt(mu/semi_major);
inclination =56*pi/180;                               %%% ISS orbit angle
ydot0= vcircular*cos(inclination);
zdot0= vcircular*sin(inclination);

%Intitial Conditions for Attitude and Angular Velocity
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0;theta0;psi0];
q0123_0= EulerAngles2Quaternions(ptp0);
p0=0;
q0=0;
r0=0;

state= [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];

%%%need time window

period= 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits=1;
tfinal=period*number_of_orbits;
timestep=1;
tout=0:timestep:tfinal;
stateout=zeros(length(tout),length(state));
% this is where we integrate the equations of motion %

%[tout,stateout] = ode23(@satellite,tspan,stateinitial);

%loop through time to integrate
BxBout= 0*stateout(:,1);
ByBout=BxBout;
BzBout=BxBout;
BxBm= 0*stateout(:,1);
ByBm=BxBout;
BzBm=BxBout;
pqrm=zeros(length(tout),3);

    %BxBN= 0*stateout(:,1);
    %ByBN=BxBout;
    %BzBN=BxBout;
    %pqrN=zeros(length(tout),3);

nextMagUpdate=100;
lastMagUpdate=0;

%sensor parameters
lastSensorUpdate=0;
nextSensorUpdate=1;

sensor_params

%print next
next=100;
lastPrint=0;

for idx=1:length(tout)
    %save the current state
    stateout(idx,:)=state';

    %print time intervl
    if tout(idx) > lastPrint
        disp(['Time = ',num2str(tout(idx))])
        lastPrint = lastPrint + next;
    end

    %4 function calls for RK4
    k1= satellite(tout(idx),state);
    k2= satellite(tout(idx) + timestep/2 , state + k1*timestep/2);
    k3= satellite(tout(idx) + timestep/2 , state + k2*timestep/2);
    k4= satellite(tout(idx) + timestep, state + k3*timestep);
    k=(1/6)*(k1 + 2*k2 + 2*k3 + k4); 
    state=state + k*timestep;
    
    %save the magnetic field
    BxBout(idx)=BB(1);
    ByBout(idx)=BB(2);
    BzBout(idx)=BB(3);
    BxBm(idx)=BfieldMeasured(1);
    ByBm(idx)=BfieldMeasured(2);
    BzBm(idx)=BfieldMeasured(3);
    
    %save filtered magnetic signal
    %BxBN(idx)=BfieldNav(1);
    %ByBN(idx)=BfieldNav(2);
    %BzBN(idx)=BfieldNav(3);


    %save the polluted pqr signal
    pqrm(idx,:)=pqrMeasured';

    %save the filtered pqr signal
    %pqrN(idx,:)=pqrNav';
end

disp('simulation complete')


%convert state to km
stateout(:,1:6)=stateout(:,1:6)/1000;

%extract state vector
xout=stateout(:,1);
yout=stateout(:,2);
zout=stateout(:,3);
q0123out=stateout(:,7:10);
ptpout=Quaternions2EulerAngles(q0123out);
pqrout=stateout(:,11:13);

%make earth
[X,Y,Z]=sphere(100);
X=X*R/1000;
Y=Y*R/1000;
Z=Z*R/1000;

%plot 3D orbit
fig1=figure();
set(fig1,'color','white')
plot3(xout,yout,zout,'b-','LineWidth',4)
grid on
hold on
surf(X,Y,Z,'EdgeColor','none');
axis equal


%%%Plot X,Y,Z as a function of time
fig2 = figure();
set(fig2,'color','white')
plot(tout,xout,'b-','LineWidth',2)
hold on
grid on
plot(tout,yout,'r-','LineWidth',2)
plot(tout,zout,'g-','LineWidth',2)
xlabel('Time (sec)')
ylabel('Altitude (km)')


%plot magnetic field
fig3=figure();
set(fig3,'color','white')
plot(tout,BxBout,'b-','LineWidth',2)  %actual
hold on
grid on
plot(tout,ByBout,'y-','LineWidth',2)  %actual
plot(tout,BzBout,'g-','LineWidth',2)  %actual
plot(tout,BxBm,'b--','LineWidth',2)   %measured
plot(tout,ByBm,'y--','LineWidth',2)   %measured
plot(tout,BzBm,'g--','LineWidth',2)   %measured
%plot(tout,BxBN,'r-','LineWidth',2)   
%plot(tout,ByBN,'m-','LineWidth',2)   
%plot(tout,BzBN,'k-','LineWidth',2)  


 
xlabel('Time (sec)')
ylabel('Mag field (T)')
legend ('X','Y','Z')

%plot norm
Bnorm=sqrt(BxBout.^2 + ByBout.^2 + BzBout.^2);
fig4=figure();
set(fig4,'color','white')
plot(tout,Bnorm,'LineWidth',2)
grid on

%plot Euler Angles
fig5 = figure();
set(fig5,'color','white')
plot(tout,ptpout,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Euler Angles (rad)')

%plot angular velocity
fig6 = figure();
set(fig6,'color','white')
plot(tout,pqrout,'LineWidth',2)    % actual 
hold on
plot(tout,pqrm,'--','LineWidth',2) % measured
%plot(tout,pqrN,'LineWidth',2) 
grid on 
xlabel('Time (sec)')
ylabel('Angular velocity (rad/s)')


toc