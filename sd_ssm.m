%% ================
% Drive Parameters
% ================

drivesNumber = 3;

% Transmission type: 1 for simple leadscrew, 2 for antbacklash leadscrew, 3 for worm drive
type = [3; 3; 1];

% Geometric parameters in SI
lead = [0; 0; 0.0012192];
leadAngle = [0; 0; 0];
threadAngle = [40*pi/180; 40*pi/180; 40*pi/180];
inElemDia = [0.010; 0.010; 0.003175];
inElemEffDia = [0; 0; 0];
outElemDia = [0.030; 0.030; 0];
outElemEffDia = [0; 0; 0];
reductionRatio = [120/1; 120/1; 0]; 

% Antibacklash preloading
springForce = [0; 0; 0];

% Friction parameters
inSupBreak = [5.5e-4; 5.5e-4; 5.5e-4];
inSupVisc = [1.15e-5; 1.15e-5; 3.5*10^-6];
outSupBreak = [4e-3; 4e-3; 4e-3];
outSupVisc = [2.2e-12; 2.2e-12; 2.2e-12];
kinFriction = [0.12; 0.12; 0.14];

strbInf = [0; 0; 0];
strbVelocity = [0; 0; 0];
risingCst = [0; 0; 0];
memoryCst = [0; 0; 0];

% Inertial Parameters in SI
inElemInertia = [6.5*10^-7; 6.5*10^-7; 0.99*10^-7];
mnOutElemInertia = [8.5*10^-6; 8.5*10^-6; 0.001]; %
secOutElemInertia = [0; 0; 0]; %

% Actuation Parameters
nominalTorque = [9.84/1000; 9.84/1000; 9.84/1000]; % mNm/mA
torqueCst = [4.78*10^-6; 4.78*10^-6; 4.78*10^-6]; % mNm/mA
encoderCounts = [1024; 1024; 1024];

for i = 1:drivesNumber
    drive.number(i).geometry.type = type(i);
    drive.number(i).geometry.lead = lead(i);
    drive.number(i).geometry.leadAngle = leadAngle(i);
    drive.number(i).geometry.inElemDia = inElemDia(i);
    drive.number(i).geometry.inElemEffDia = inElemEffDia(i);
    drive.number(i).geometry.outElemDia = outElemDia(i);
    drive.number(i).geometry.outElemEffDia = outElemEffDia(i);
    drive.number(i).geometry.reductionRatio = reductionRatio(i);
    drive.number(i).geometry.threadAngle = threadAngle(i);
    
    drive.number(i).preload.springForce = springForce(i);
    
    drive.number(i).friction.inSupBreak = inSupBreak(i);
    drive.number(i).friction.inSupVisc = inSupVisc(i);
    drive.number(i).friction.outSupBreak = outSupBreak(i);
    drive.number(i).friction.outSupVisc = outSupVisc(i);
    drive.number(i).friction.kinFriction = kinFriction(i);
    drive.number(i).friction.strbInf = strbInf(i);
    drive.number(i).friction.strbVelocity = strbVelocity(i);
    drive.number(i).friction.risingCst = risingCst(i);
    drive.number(i).friction.memoryCst = memoryCst(i);
    
    drive.number(i).inertia.inElemInertia = inElemInertia(i);
    drive.number(i).inertia.mnOutElemInertia = mnOutElemInertia(i);
    drive.number(i).inertia.secOutElemInertia = secOutElemInertia(i);
    
    drive.number(i).actuation.nominalTorque = nominalTorque(i);
    drive.number(i).actuation.torqueCst = torqueCst(i);
    drive.number(i).actuation.encoderCounts = encoderCounts(i);
end


% DriveNo1 : Worm Drive (Joint 1)
drv1 = Transmission;
drv1.geometry = drive.number(1).geometry;
drv1.friction = drive.number(1).friction;
drv1.inertia = drive.number(1).inertia;
drv1.actuation = drive.number(1).actuation;

% DriveNo2 : Worm Drive (Joint 2)
drv2 = Transmission;
drv2.geometry = drive.number(2).geometry;
drv2.friction = drive.number(2).friction;
drv2.inertia = drive.number(2).inertia;
drv2.actuation = drive.number(2).actuation;

% DriveNo4 : Simple Lead Screw Drive (Joint 4)
drv4 = Transmission;
drv4.geometry = drive.number(3).geometry;
drv4.friction = drive.number(3).friction;
drv4.inertia = drive.number(3).inertia;
drv4.actuation = drive.number(3).actuation;
