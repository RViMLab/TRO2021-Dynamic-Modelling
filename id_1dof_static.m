clear; clc; sd_1dof;

%% Drive to identify
drv = driveNo3;
mass = 0.096; % constant payload
[loadInertia, payload] = extLoad(mass, drv.geometry.type);
constants = [Transmission.getGamma(drv.geometry.inElemDia, drv.geometry.outElemDia);...
    Transmission.getGamma(drv.geometry.inElemEffDia, drv.geometry.outElemEffDia);...
    drv.geometry.leadAngle; drv.geometry.pressureAngle; payload];

%% Experimental Data
files = ['experiments\driveNo' num2str(drv.geometry.type) '\steady_state\' num2str(mass) 'kg' '\*.csv'];
map = getMap(files, drv.actuation.nominalTorque);
velocity = map(:,1); % rad/sec
torque = map(:,2); % Nm

figure('Name', 'Exp data')
scatter(velocity*30/pi, torque*1000); grid
xlimits = [floor(min(velocity*30/pi)), ceil(max(velocity*30/pi))]; % rpm
ylimits = [-ceil(max(abs(torque*1000))), ceil(max(abs(torque*1000)))]; % mNm
axis([xlimits, ylimits]);

%% Simple Coulomb Fit
preload = range.drvType(drv.geometry.type).preload;
inSupBreak = range.drvType(drv.geometry.type).inSupBreak;
inSupVisc = range.drvType(drv.geometry.type).inSupVisc;
outSupBreak = range.drvType(drv.geometry.type).outSupBreak;
outSupVisc = range.drvType(drv.geometry.type).outSupVisc;
kinFriction = range.drvType(drv.geometry.type).kinFriction;

alpha0 = [constants(:); preload(1); inSupBreak(1); inSupVisc(1); outSupBreak(1); outSupVisc(1); kinFriction(1)]; 
lb = [constants(:); preload(2); inSupBreak(2); inSupVisc(2); outSupBreak(2); outSupVisc(2); kinFriction(2)];
ub = [constants(:); preload(3); inSupBreak(3); inSupVisc(3); outSupBreak(3); outSupVisc(3); kinFriction(3)];

options=optimset('Display', 'Iter', 'TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 10000, 'MaxIter', 10000);

alphaC = lsqcurvefit(@(alphaC,velocity) coulomb(alphaC,velocity), alpha0, velocity, torque, lb, ub, options);
alphaCopt = display(alphaC)

figure('Name', 'Simple fit')
plot(velocity*30/pi, torque*1000, 'ko'); hold on;
plot(velocity*30/pi, coulomb(alphaC, velocity)*1000, 'b-');
axis([xlimits, ylimits]); hold off

%% Stribeck Fit
preload = range.drvType(drv.geometry.type).preload;
inSupBreak = range.drvType(drv.geometry.type).inSupBreak;
inSupVisc = range.drvType(drv.geometry.type).inSupVisc;
outSupBreak = range.drvType(drv.geometry.type).outSupBreak;
outSupVisc = range.drvType(drv.geometry.type).outSupVisc;
kinFriction = range.drvType(drv.geometry.type).kinFriction;
strbInf = range.drvType(drv.geometry.type).strbInf;
strbVelocity = range.drvType(drv.geometry.type).strbVelocity;

alpha0 = [constants(:); preload(1); inSupBreak(1); inSupVisc(1); outSupBreak(1); outSupVisc(1);...
    kinFriction(1); strbInf(1); strbVelocity(1)]; 
lb = [constants(:); preload(2); inSupBreak(2); inSupVisc(2); outSupBreak(2); outSupVisc(2);...
    kinFriction(2); strbInf(2); strbVelocity(2)];
ub = [constants(:); preload(3); inSupBreak(3); inSupVisc(3); outSupBreak(3); outSupVisc(3);
    kinFriction(3); strbInf(3); strbVelocity(3)];

options=optimset('Display', 'Iter', 'TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 10000, 'MaxIter', 10000);

alphaS = lsqcurvefit(@(alphaS,velocity) stribeck(alphaS,velocity), alpha0, velocity, torque, lb, ub, options);
alphaSopt = display(alphaS)

figure('Name', 'Stribeck fit')
plot(velocity*30/pi, torque*1000, 'ko'); hold on;
plot(velocity*30/pi, stribeck(alphaS, velocity)*1000, 'b-');
axis([xlimits, ylimits]); hold off


%% functions

function estTorque = coulomb(alpha, velocity)
    
    % constants
    gamma0 = alpha(1);
    gamma = alpha(2);
    lambda = alpha(3);
    thetaN = alpha(4);
    load = alpha(5);
    
    % parameters
    preload = alpha(6);
    inSupBreak = alpha(7);
    inSupVisc = alpha(8);
    outSupBreak = alpha(9);
    outSupVisc = alpha(10);
    kinFriction = alpha(11);
    
    reposeAngleEquiv = atan(kinFriction/cos(thetaN));
    uphill = tan(lambda + reposeAngleEquiv);
    downhill = tan(lambda - reposeAngleEquiv);
    psi0 = tan(lambda);
    
    velocityNeg = velocity(velocity < 0);
    velocityPos = velocity(velocity >= 0);
    
    if load == 0 && preload == 0
        estTorqueNeg = inSupBreak*sign(velocityNeg) + inSupVisc*velocityNeg...
            + gamma*uphill.*(outSupBreak*sign(velocityNeg) + outSupVisc*gamma0*psi0*velocityNeg);
        estTorquePos = inSupBreak*sign(velocityPos) + inSupVisc*velocityPos...
            + gamma*uphill.*(outSupBreak*sign(velocityPos) + outSupVisc*gamma0*psi0*velocityPos);
    else
        estTorqueNeg = -gamma*uphill*load...
            + inSupBreak*sign(velocityNeg) + inSupVisc*velocityNeg...
            + gamma*uphill.*(outSupBreak*sign(velocityNeg) + outSupVisc*gamma0*psi0*velocityNeg)...
            + gamma*preload*(downhill - uphill);
        estTorquePos = -gamma*downhill*load...
            + inSupBreak*sign(velocityPos) + inSupVisc*velocityPos...
            + gamma*downhill.*(outSupBreak*sign(velocityPos) + outSupVisc*gamma0*psi0*velocityPos)...
            + gamma*preload*(uphill - downhill);
    end

    estTorque = [estTorqueNeg; estTorquePos]; 

end

function estTorque = stribeck(alpha, velocity)
    
    % constants
    gamma0 = alpha(1);
    gamma = alpha(2);
    lambda = alpha(3);
    thetaN = alpha(4);
    load = alpha(5);
    
    % parameters
    preload = alpha(6);
    inSupBreak = alpha(7);
    inSupVisc = alpha(8);
    outSupBreak = alpha(9);
    outSupVisc = alpha(10);
    kinFriction = alpha(11);
    strbFriction = alpha(12);
    strbVelocity = alpha(13);
    
    velocityNeg = velocity(velocity < 0);
    velocityPos = velocity(velocity > 0);
    
    % exponential fit
    frictionCoeffNeg = kinFriction + strbFriction*exp(-abs(velocityNeg)/strbVelocity).^2;
    frictionCoeffPos = kinFriction + strbFriction*exp(-abs(velocityPos)/strbVelocity).^2;
    
%     % quadratic fit
%     frictionCoeffNeg = kinFriction + strbFriction./(1 + (abs(velocityNeg)/strbVelocity).^2);
%     frictionCoeffPos = kinFriction + strbFriction./(1 + (abs(velocityPos)/strbVelocity).^2);
    
    reposeAngleEquivNeg = atan(frictionCoeffNeg/cos(thetaN));
    reposeAngleEquivPos = atan(frictionCoeffPos/cos(thetaN));
    
    uphillNeg = tan(lambda + reposeAngleEquivNeg);
    uphillPos = tan(lambda + reposeAngleEquivPos);
    downhillNeg = tan(lambda - reposeAngleEquivNeg);
    downhillPos = tan(lambda - reposeAngleEquivPos);
    
    psi0 = tan(lambda);
    
    
    if load == 0 && preload == 0
        estTorqueNeg = inSupBreak*sign(velocityNeg) + inSupVisc*velocityNeg...
            + gamma*uphillNeg.*(outSupBreak*sign(velocityNeg) + outSupVisc*gamma0*psi0*velocityNeg);
        estTorquePos = inSupBreak*sign(velocityPos) + inSupVisc*velocityPos...
            + gamma*uphillPos.*(outSupBreak*sign(velocityPos) + outSupVisc*gamma0*psi0*velocityPos);
    else
        estTorqueNeg = -gamma*uphillNeg*load...
            + inSupBreak*sign(velocityNeg) + inSupVisc*velocityNeg...
            + gamma*uphillNeg.*(outSupBreak*sign(velocityNeg) + outSupVisc*gamma0*psi0*velocityNeg)...
            + gamma*preload*(downhillNeg - uphillNeg);
        estTorquePos = -gamma*downhillPos*load...
            + inSupBreak*sign(velocityPos) + inSupVisc*velocityPos...
            + gamma*downhillPos.*(outSupBreak*sign(velocityPos) + outSupVisc*gamma0*psi0*velocityPos)...
            + gamma*preload*(uphillPos - downhillPos);
    end
    
    estTorque = [estTorqueNeg; estTorquePos]; 

end

function map = getMap(files, nominalTorque)

    list = dir(files);

    meanVelocities = zeros(length(list),1);
    meanTorques = zeros(length(list),1);

    for i = 1:length(list)
        
        dirName = list(i).folder;
        fileName = list(i).name;
        
        [~, ~, ~, ~, velocityAvg, torqueAvg] = getEposData(dirName, fileName, nominalTorque);
        
        meanVelocities(i) = mean(velocityAvg.Data);
        meanTorques(i) = mean(torqueAvg.Data);

    end

    [meanVelocities, order] = sort(meanVelocities);
    meanTorques = meanTorques(order);
    
    meanPos = meanVelocities(meanVelocities>0);
    meanNeg = meanVelocities(meanVelocities<0);
    
    if ~isempty(meanPos) && ~isempty(meanNeg)
        meanVelocities = [meanNeg; -eps; eps; meanPos];
        idx = find(meanVelocities > 0, 1);
        meanTorques = [meanTorques(1:idx-2); meanTorques(idx-2);...
            meanTorques(idx-1); meanTorques(idx-1:end)];
    elseif isempty(meanPos) && ~isempty(meanNeg)
        meanVelocities = [meanNeg; -eps];
        meanTorques = [meanTorques; meanTorques(end)];
    elseif ~isempty(meanPos) && isempty(meanNeg)
        meanVelocities = [eps; meanPos];
        meanTorques = [meanTorques(1); meanTorques];
    else
        meanVelocities = [];
        meanTorques = [];
    end
    

    map = [meanVelocities meanTorques];

end

function optParam = display(alpha)

    
    preload = alpha(6);
    inSupBreak = alpha(7);
    inSupVisc = alpha(8);
    outSupBreak = alpha(9);
    outSupVisc = alpha(10);
    kinFriction = alpha(11);
    
    if numel(alpha) == 11
        
        optParam = struct('preload', preload, 'inSupBreak', inSupBreak, 'inSupVisc', inSupVisc,...
            'outSupBreak', outSupBreak, 'outSupVisc', outSupVisc, 'kinFriction', kinFriction,...
            'strbInf', 0, 'strbVelocity', 0, 'risingCst', 0, 'memoryCst', 0);
        
    else
        strbInf = alpha(12);
        strbVelocity = alpha(13);
        
        optParam = struct('preload', preload, 'inSupBreak', inSupBreak, 'inSupVisc', inSupVisc,...
            'outSupBreak', outSupBreak, 'outSupVisc', outSupVisc, 'kinFriction', kinFriction,...
            'strbInf', strbInf, 'strbVelocity', strbVelocity, 'risingCst', 0, 'memoryCst', 0);
    end

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