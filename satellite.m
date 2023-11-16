function dstatedt= satellite(t,state)



global BB m I invI nextMagUpdate lastMagUpdate lastSensorUpdate nextSensorUpdate BfieldMeasured pqrMeasured 
global BfieldNav pqrNav

inertia

x= state(1);
y= state(2);
z= state(3);
%xdot= state(4);
%ydot= state(5);
%zdot= state(6);
q0123 = state(7:10);
%ptp = Quaternions2EulerAngles(q0123')';
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);


%translational kinematics
vel=state(4:6);

%Rotational Kinematics
PQRMAT = [0 -p -q -r ; p 0 r -q ; q -r 0 p ; r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;

%Gravity model
planet
r=state(1:3);  % r=[x;y;z]
rho=norm(r);
rhat=r/rho;
Fgrav=-G*(M*(m/rho^2)*rhat);

%call the magnetic field model
if t>=lastMagUpdate
    lastMagUpdate=lastMagUpdate + nextMagUpdate;


    %convert cartesian x,y,z to lat, long, alt
    phiE=0;
    thetaE=acos(z/rho);
    psiE=atan2(y,x);
    latitude=90-thetaE*180/pi;
    longitude=psiE*180/pi;
    rhokm=(rho)/1000; 
    
    [BN,BE,BD]=igrf('01-Jan-2000',latitude,longitude,rhokm,'geocentric');
      
    %Convert NED (North East Down to X,Y,Z in ECI frame)
    %First we need to create a rotation matrix from the NED frame to the inertial frame
    
    BNED=[BN;BE;BD];    
    BI = TIB(phiE,thetaE+pi,psiE)*BNED;
    BB=TIBquat(q0123)'*BI;

    %convert to Tesla
    BB=BB*1e-9;


end

%sensor block
if t>=lastSensorUpdate
    lastSensorUpdate=lastSensorUpdate + nextSensorUpdate;
    [BfieldMeasured,pqrMeasured]=Sensor(BB, pqr);

    %navigation block
    %[BfieldNav,pqrNav] = Navigation(BfieldMeasured, pqrMeasured);

end


%translational dynamics
F=Fgrav;
accel=F/m;

%magtorquer model
LMN_magtorquers=[0;0;0];

%Rotational Dynamics
H = I*pqr;   %angular momentum
pqrdot = invI*(LMN_magtorquers - cross(pqr,H));

%return derivatives vector
dstatedt=[vel;accel;q0123dot;pqrdot];
