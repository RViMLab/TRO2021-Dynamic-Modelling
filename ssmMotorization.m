clear; clc; 
ssmBuild;
sd_ssm;

%% Initialization and Simulation Parameters

initConfig = [0 0 0 0];

% Joint to be simulated and max values for acceleration/velocity/position
jointIdx = 2;
maxAccel = 10; 
maxVel = 2;
posLim = [pi/2 -pi/2];

% jointIdx = 4;
% maxAccel = 0.1;
% maxVel = 0.03;
% posLim = [0.03 -0.03];

[timeStamps, simTime] = getTimes(maxAccel, maxVel, posLim);
stepSize = 0.01;

%% Inverse Dynamics Simulation
tic; sim('simINVD_ssm.slx'); toc

%% Plots

figure('Name', 'Robot Joints')
subplot(511)
plot(jointAccel); title(''); ylabel('Acceleration')
subplot(512)
plot(jointVel); title(''); ylabel('Velocity')
subplot(513)
plot(jointPos); title(''); ylabel('Position')
subplot(514)
plot(payload); title(''); ylabel('Payload')
subplot(515)
plot(jointTorq); title(''); ylabel('Torque')

figure('Name', 'Motor Load Curves')
subplot(311)
plot(1000*squeeze(torque1.data),30/pi*squeeze(velocity1.data));
title('Motor 1'); xlabel('Torque [mNm]'); ylabel('Velocity [r/min]')
subplot(312)
plot(1000*squeeze(torque2.data),30/pi*squeeze(velocity2.data));
title('Motor 2'); xlabel('Torque [mNm]'); ylabel('Velocity [r/min]')
subplot(313)
plot(1000*squeeze(torque4.data),30/pi*squeeze(velocity4.data));
title('Motor 4'); xlabel('Torque [mNm]'); ylabel('Velocity [r/min]')

figure('Name', 'Motor Power Variables')
subplot(231)
plot(1000*torque1); title('Motor 1'); ylabel('Torque [mNm]');
subplot(234)
plot(30/pi*velocity1); title('Motor 1'); ylabel('Velocity [r/min]')
subplot(232)
plot(1000*torque2); title('Motor 2'); ylabel('Torque [mNm]');
subplot(235)
plot(30/pi*velocity2); title('Motor 2'); ylabel('Velocity [r/min]')
subplot(233)
plot(1000*torque4); title('Motor 4'); ylabel('Torque [mNm]');
subplot(236)
plot(30/pi*velocity4); title('Motor 4'); ylabel('Velocity [r/min]')

%% functions

function [timeStamps, simTime] = getTimes(maxAccel, maxVel, posLim)

% This function returns the times when the velocity changes.
% The timestamps are used in simINVD_ssm.slx to create a motion profile
% based on the maximum values for position/velocity/acceleration.
    
    sTime = maxVel/maxAccel;
    cTime = abs(posLim)/maxVel - sTime;
    if ~isempty(cTime(cTime<0))
        error('Inconsistent maxAccel/maxVel for given range')
    end
    motionTime = 8*sTime + 2*cTime(1) + 2*cTime(2);
    simTime = rndeven(motionTime);
    edgeTime = (simTime-motionTime)/3;
    
    t1 = edgeTime;
    t2 = t1 + sTime;
    t3 = t2 + cTime(1);
    t4 = t3 + 2*sTime;
    t5 = t4 + cTime(1);
    t6 = t5 + sTime;
    t7 = t6 + edgeTime;
    t8 = t7 + sTime;
    t9 = t8 + cTime(2);
    t10 = t9 + 2*sTime;
    t11 = t10 + cTime(2);
    t12 = t11 + sTime;
    
    timeStamps = [t1 t2 t3 t4 t5 t6 t7 t8 t9 t10 t11 t12];
    
end

function y = rndeven(x)

    x = ceil(x);
    x(x <= 1) = 2;
    y = mod(x,2)+x;
    
end
