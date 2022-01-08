clear; clc; sd_1dof;

%% Drive to identify
global drv;
drv = driveNo3;
mass = 0.096; % constant payload
[loadInertia, payload] = extLoad(mass, drv.geometry.type);

%% Experimental Data
driveDir = ['\experiments\driveNo' num2str(drv.geometry.type) '\start_stop'];
dir = [pwd driveDir];
file = '200.csv';

[recordedData, samplingPeriod, simTime, ~, velocityAvg, actrTorqueAvg, velocity, actrTorque] = ...
    getEposData(dir, file, drv.actuation.nominalTorque);

actrTorqueAvg = timeseries(medfilt1(actrTorqueAvg.data, 5, 'truncate'), actrTorqueAvg.time);
velocityAvg = timeseries(medfilt1(velocityAvg.data, 3, 'truncate'), velocityAvg.time);
[velocityInit, frictionInit, dwellTimeInit] = getInitValues(velocityAvg.data, drv.friction.kinFriction,...
    drv.friction.strbInf, drv.friction.strbVelocity);

%% Dynamic parameter identification
inSupBreak = range.drvType(drv.geometry.type).inSupBreak;
inSupVisc = range.drvType(drv.geometry.type).inSupVisc;
outSupBreak = range.drvType(drv.geometry.type).outSupBreak;
outSupVisc = range.drvType(drv.geometry.type).outSupVisc;
kinFriction = range.drvType(drv.geometry.type).kinFriction;
strbInf = range.drvType(drv.geometry.type).strbInf;
strbVelocity = range.drvType(drv.geometry.type).strbVelocity;
risingCst = range.drvType(drv.geometry.type).risingCst;
memoryCst = range.drvType(drv.geometry.type).memoryCst;

sigma0 = [inSupBreak(1); inSupVisc(1); outSupBreak(1); outSupVisc(1); kinFriction(1); strbInf(1);...
    strbVelocity(1); risingCst(1); memoryCst(1)];
lb = [inSupBreak(2); inSupVisc(2); outSupBreak(2); outSupVisc(2); kinFriction(2); strbInf(2);...
    strbVelocity(2); risingCst(2); memoryCst(2)];
ub = [inSupBreak(3); inSupVisc(3); outSupBreak(3); outSupVisc(3); kinFriction(3); strbInf(3);...
    strbVelocity(3); risingCst(3); memoryCst(3)];

options = optimset('Display', 'Iter', 'TolFun', 1e-55, 'TolX', 1e-55, 'MaxFunEvals', 100000, 'MaxIter', 5);

sigma = lsqnonlin(@(sigma) getTorqueCost(sigma, actrTorqueAvg.Data), sigma0, lb, ub, options);
sigmaOpt = display(sigma)

%% Simulation

% Optimal fit
drv.friction = sigmaOpt;
tic; sim('simINVD_1dof.slx'); toc
figure(1)
plot(actrTorqueAvg*1000); hold on;
xlabel('Time [s]'); ylabel('Torque [mNm]');title('')
plot(torqueSim*1000); hold off;

%% functions

function error = getTorqueCost(sigma, expData)
    
    global drv
    drv.friction.inSupBreak = sigma(1);
    drv.friction.inSupVisc = sigma(2);
    drv.friction.outSupBreak = sigma(3);
    drv.friction.outSupVisc = sigma(4);
    drv.friction.kinFriction = sigma(5);
    drv.friction.strbInf = sigma(6);
    drv.friction.strbVelocity = sigma(7);
    drv.friction.risingCst = sigma(8);
    drv.friction.memoryCst = sigma(9);
    
    sim('simINVD_1dof.slx')
    
    simData = squeeze(torqueSim.Data);
    Fx1 = griddedInterpolant(simData);
    Fx2 = griddedInterpolant(expData);
    simData = Fx1(linspace(1,numel(simData),numel(expData))');
    expData = Fx2(linspace(1,numel(expData),numel(expData))');
    
    error = simData - expData;
    
end

function optParam = display(sigma)


    inSupBreak = sigma(1);
    inSupVisc = sigma(2);
    outSupBreak = sigma(3);
    outSupVisc = sigma(4);
    kinFriction = sigma(5);
    strbInf = sigma(6);
    strbVelocity = sigma(7);
    risingCst = sigma(8);
    memoryCst = sigma(9);
    
    optParam = struct('inSupBreak', inSupBreak, 'inSupVisc', inSupVisc, 'outSupBreak', outSupBreak,...
        'outSupVisc', outSupVisc, 'kinFriction', kinFriction, 'strbInf', strbInf,...
        'strbVelocity', strbVelocity, 'risingCst', risingCst, 'memoryCst', memoryCst);
    
end

function [loadInertia, payload] = extLoad(mass, driveType)

    gravity = 9.812;
    
    if driveType ~= 3
        loadInertia = mass;
        payload = gravity*mass;
    else
        pulleyRad = 0.02;
        loadInertia = mass*pulleyRad^2;
        payload = gravity*mass*pulleyRad;
    end

end

function [velocityInit, frictionInit, dwellTimeInit] = getInitValues(expVelocity, kinFriction, strbInf,...
    strbVelocity)
    
    velocityInit = mean(expVelocity(1:5));
    frictionInit = Transmission.getFrictionCoeff(velocityInit, kinFriction, strbInf, strbVelocity);
    
    if velocityInit == 0
        dwellTimeInit = 10;
    else
        dwellTimeInit = 0;
    end
    
end