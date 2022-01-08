clear; clc;

%% ================
% Drive Parameters
% ================

drivesNumber = 3; 

% Transmission type: 1 for simple leadscrew, 2 for antbacklash leadscrew, 3 for worm drive
type = [1; 2; 3];

% Geometric parameters in SI
lead = [0.002; 0.002; 0];
leadAngle = [0; 0; 2.8625*pi/180];
threadAngle = [30*pi/180; 30*pi/180; 40*pi/180];
inElemDia = [0.012; 0.012; 0.010];
inElemEffDia = [0.011; 0.011; 0];
outElemDia = [0; 0; 0];
outElemEffDia = [0; 0; 0];
reductionRatio = [0; 0; 60/1]; 

% Antibacklash preloading force in Newton
springForce = [0; 0.8; 0];

% Friction parameters in SI
inSupBreak = [1e-3; 2e-3; 5.5e-4];
inSupVisc = [3e-5; 3e-5; 1.15e-5];
outSupBreak = [3e-2; 2e-2; 4e-3];
outSupVisc = [1e-5; 2e-10; 2.2e-12];
kinFriction = [0.27; 0.20; 0.12];

strbInf = [0.1; 0.03; 0.06];
strbVelocity = [6.4; 0; 0.688];

risingCst = [0; 0.1; 1.5];
memoryCst = [0; 0.05; 0.25];

% Inertial Parameters in SI
inElemInertia = [2.186*10^-6; 2.186*10^-6; 6.5*10^-7];
mnOutElemInertia = [0.180; 0.180; 8.5*10^-6]; %
secOutElemInertia = [0; 0.05; 0]; %

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


% DriveNo1 : Simple Lead Screw Drive
driveNo1 = Transmission;
driveNo1.geometry = drive.number(1).geometry;
driveNo1.friction = drive.number(1).friction;
driveNo1.preload = drive.number(1).preload;
driveNo1.inertia = drive.number(1).inertia;
driveNo1.actuation = drive.number(1).actuation;

% DriveNo2 : Antibacklash Lead Screw Drive
driveNo2 = Transmission;
driveNo2.geometry = drive.number(2).geometry;
driveNo2.friction = drive.number(2).friction;
driveNo2.preload = drive.number(2).preload;
driveNo2.inertia = drive.number(2).inertia;
driveNo2.actuation = drive.number(2).actuation;

% DriveNo3 : Worm Drive
driveNo3 = Transmission;
driveNo3.geometry = drive.number(3).geometry;
driveNo3.friction = drive.number(3).friction;
driveNo3.inertia = drive.number(3).inertia;
driveNo3.actuation = drive.number(3).actuation;


%% Parameters range
% range = [init value; lower limit; upper limit]

range.drvType(1).preload = [0; 0; 0];
range.drvType(1).inSupBreak = [2e-4; 1e-3; 1e-2];
range.drvType(1).inSupVisc = [1e-5; 1e-5; 1e-3];
range.drvType(1).outSupBreak = [1e-5; 1e-3; 5e-2];
range.drvType(1).outSupVisc = [0; 0; 1e-5];
range.drvType(1).kinFriction = [0.18; 0.20; 0.21];
range.drvType(1).strbInf = [0.05; 0; 0.08];
range.drvType(1).strbVelocity = [5; 0; 15];
range.drvType(1).risingCst = [0.1; 0; 0];
range.drvType(1).memoryCst = [0; 0; 0];

range.drvType(2).preload = [0.75; 0.5; 1.5];
range.drvType(2).inSupBreak = [2e-4; 1e-5; 1e-2];
range.drvType(2).inSupVisc = [1e-5; 1e-5; 1e-3];
range.drvType(2).outSupBreak = [1e-5; 1e-5; 5e-2];
range.drvType(2).outSupVisc = [0; 0; 1e-5];
range.drvType(2).kinFriction = [0.20; 0.15; 0.27];
range.drvType(2).strbInf = [0.1; 0; 0.2];
range.drvType(2).strbVelocity = [5; 0; 15];
range.drvType(2).risingCst = [0.1; 0; 0];
range.drvType(2).memoryCst = [0; 0; 0];

range.drvType(3).preload = [0; 0; 0];
range.drvType(3).inSupBreak = [2e-4; 1e-4; 1e-3];
range.drvType(3).inSupVisc = [1e-5; 1e-5; 1e-3];
range.drvType(3).outSupBreak = [1e-5; 1e-5; 1e-2];
range.drvType(3).outSupVisc = [0; 0; 1e-5];
range.drvType(3).kinFriction = [0.15; 0.12; 0.16];
range.drvType(3).strbInf = [0.05; 0; 0.1];
range.drvType(3).strbVelocity = [0.5; 0; 1];
range.drvType(3).risingCst = [0.1; 0; 0];
range.drvType(3).memoryCst = [0.15; 0; 0.25];