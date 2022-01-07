classdef Transmission < matlab.System
    
    
    properties
        geometry = struct('type', 0, 'lead', 0, 'leadAngle', 0, 'inElemDia', 0, 'inElemEffDia', 0,...
            'outElemDia', 0, 'outElemEffDia', 0, 'reductionRatio', 0, 'threadAngle', 0, 'pressureAngle', 0)
        preload = struct('springForce', 0)
        friction = struct('inSupBreak', 0, 'inSupVisc', 0, 'outSupBreak', 0, 'outSupVisc', 0,...
            'kinFriction', 0, 'strbInf', 0, 'strbVelocity', 0, 'risingCst', 0, 'memoryCst', 0)
        inertia = struct('inElemInertia', 0, 'mnOutElemInertia', 0, 'secOutElemInertia', 0)
        actuation = struct('nominalTorque', 0, 'torqueCst', 0, 'encoderCounts', 0)
    end
    
    properties(Hidden, Constant)
        zeroTolerance = 0.005;
    end
    
    properties(Access = private)
        gamma0
        gamma
        psi0
    end
    
    
    methods(Static = true)
        %%
        
        function LSDgeometry = setLSDgeometry(lead, leadAngle, screwDiameter, reductionRatio)
            
            % geometric equations:
            % Eq.(1) tan(leadAngle) = lead/pi/screwDiameter
            % Eq.(2) reductionRatio = 2*pi/lead ( = 1/(screwDiamaeter/2)/tan(leadAngle) )
            
            eq1variables = [screwDiameter leadAngle lead];
            eq2variables = [reductionRatio lead];
            variables = [lead leadAngle screwDiameter reductionRatio];
            
            if numel(variables(variables~=0)) < 2
                error('Geometry error: inadequate data provided')
            elseif all(variables ~= 0)
                tolerance = 10^-9;
                if abs(tan(leadAngle)-lead/pi/screwDiameter) > tolerance ...
                        || abs(reductionRatio-2*pi/lead) > tolerance 
                    error('Geometry error: inadequate data provided')
                end
            elseif numel(eq1variables(eq1variables~=0)) > 2 || numel(eq2variables(eq2variables~=0)) > 1 ...
                    || numel(variables(variables~=0)) > 2
                error('Geometry error: inadequate data provided')
                % 1 restriction when 2 variables are provided
            else  % 4 choose 2 combinations minus 1 restriction (5 different cases)
                if (lead && leadAngle) ~= 0
                    screwDiameter = lead/pi/tan(leadAngle);
                    reductionRatio = 2*pi/lead;
                elseif (lead && screwDiameter) ~= 0
                    leadAngle = atan(lead/pi/screwDiameter);
                    reductionRatio = 2*pi/lead;
                elseif (leadAngle && screwDiameter) ~= 0
                    lead = tan(leadAngle)*pi*screwDiameter;
                    reductionRatio = 2*pi/lead;
                elseif (leadAngle && reductionRatio) ~= 0
                    screwDiameter = 1/(reductionRatio/2)/tan(leadAngle);
                    lead = tan(leadAngle)*pi*screwDiameter;
                elseif (screwDiameter && reductionRatio) ~= 0
                    leadAngle = atan(1/(screwDiameter/2)/reductionRatio);
                    lead = tan(leadAngle)*pi*screwDiameter;
                else
                end
            end
            
            LSDgeometry = [lead; leadAngle; screwDiameter; reductionRatio];
                        
        end
        
        function WDgeometry = setWDgeometry(lead, leadAngle, wormDiameter, gearDiameter, reductionRatio)
            
            % geometric equations:
            % Eq.(1) tan(leadAngle) = lead/pi/wormDiameter
            % Eq.(2) reductionRatio = pi*gearDiameter/lead ( = gearDiameter/wormDiameter/tan(leadAngle) )
            
            eq1variables = [wormDiameter leadAngle lead];
            eq2variables = [reductionRatio gearDiameter lead];
            variables = [lead leadAngle wormDiameter gearDiameter reductionRatio];
                       
            if numel(variables(variables~=0)) < 3
                error('Geometry error: inadequate data provided')
            elseif all(variables~=0)
                tolerance = 10^-9;
                if abs(tan(leadAngle)-lead/pi/wormDiameter) > tolerance ...
                        || abs(reductionRatio - pi*gearDiameter/lead) > tolerance 
                    error('Geometry error: inadequate data provided')
                end
            elseif numel(eq1variables(eq1variables~=0)) > 2 || numel(eq2variables(eq2variables~=0)) > 2 ...
                    || numel(variables(variables~=0)) > 3
                error('Geometry error: inadequate data provided')
                % 2 restrictions when 3 variables are provided
            else
                % 5 choose 3 combinations minus 2 restrictions (8 different cases)
                if (lead && leadAngle && gearDiameter) ~= 0
                    wormDiameter = lead/pi/tan(leadAngle);
                    reductionRatio = gearDiameter/wormDiameter/tan(leadAngle);
                elseif (lead && leadAngle && reductionRatio) ~= 0
                    wormDiameter = lead/pi/tan(leadAngle);
                    gearDiameter = tan(leadAngle)*wormDiameter*reductionRatio;
                elseif (lead && wormDiameter && gearDiameter) ~= 0
                    leadAngle = atan(lead/pi/wormDiameter);
                    reductionRatio = gearDiameter/wormDiameter/tan(leadAngle);
                elseif (lead && wormDiameter && reductionRatio) ~= 0
                    leadAngle = atan(lead/pi/wormDiameter);
                    gearDiameter = tan(leadAngle)*wormDiameter*reductionRatio;
                elseif (leadAngle && wormDiameter && gearDiameter) ~= 0
                    lead = tan(leadAngle)*pi*wormDiameter;
                    reductionRatio = gearDiameter/wormDiameter/tan(leadAngle);
                elseif (leadAngle && wormDiameter && reductionRatio) ~= 0
                    lead = tan(leadAngle)*pi*wormDiameter;
                    gearDiameter = tan(leadAngle)*wormDiameter*reductionRatio;
                elseif (leadAngle && gearDiameter && reductionRatio) ~= 0
                    lead = pi*gearDiameter/reductionRatio;
                    wormDiameter = lead/pi/tan(leadAngle);
                elseif (wormDiameter && gearDiameter && reductionRatio) ~= 0
                    lead = pi*gearDiameter/reductionRatio;
                    leadAngle = atan(lead/pi/wormDiameter);
                else
                end
            end
            
            WDgeometry = [lead; leadAngle; wormDiameter; gearDiameter; reductionRatio];
            
        end
        
        function gamma = getGamma(inElemDia, outElemDia)
            
            if outElemDia == 0
                gamma = inElemDia/2;
            else
                gamma = inElemDia/outElemDia;
            end
            
        end
        
        function psi0 = getPsi0(leadAngle)
            
            psi0 = tan(leadAngle);
            
        end
        
        function nonCompression(preload, payload, gamma0, psi0, acceleration, velocity, mnOutElemInertia,...
                loadInertia, secOutElemInertia, outSupBreak, outSupVisc)
            
            if preload == 0
                return
            else
                if velocity == 0
                    if (preload + payload - (mnOutElemInertia + loadInertia)*gamma0*psi0*acceleration...
                            - outSupBreak*sign(acceleration)) > 0 &&...
                            (preload + secOutElemInertia*gamma0*psi0*acceleration) > 0
                        return
                    else
                        error('Antibacklash operation violated')
                    end
                else
                    if (preload + payload - (mnOutElemInertia + loadInertia)*gamma0*psi0*acceleration...
                            - outSupBreak*sign(velocity) - outSupVisc*gamma0*psi0*velocity) > 0 &&...
                            (preload + secOutElemInertia*gamma0*psi0*acceleration) > 0
                        return
                    else
                        error('Antibacklash operation violated')
                    end
                end
            end
            
        end
        
        function Dvar = getDvar(preload, payload, gamma0, psi0, tendency, acceleration, velocity,...
                mnOutElemInertia, loadInertia, outSupBreak, outSupVisc)
            
            if preload == 0
                if acceleration == 0 && velocity == 0
                    Dvar = (payload - outSupBreak*sign(tendency))*sign(tendency);
                elseif acceleration ~=0 && velocity == 0
                    Dvar = (payload - (mnOutElemInertia + loadInertia)*gamma0*psi0*acceleration...
                        - outSupBreak*sign(acceleration))*sign(acceleration);
                else
                    Dvar = (payload - (mnOutElemInertia + loadInertia)*gamma0*psi0*acceleration...
                        - outSupBreak*sign(velocity) - outSupVisc*gamma0*psi0*velocity)*sign(velocity);
                end
            else
                if acceleration == 0 && velocity == 0
                    Dvar = sign(tendency);
                elseif acceleration ~=0 && velocity == 0
                    Dvar = sign(acceleration);
                else
                    Dvar = sign(velocity);
                end
            end
            
        end
        
        function [strbFriction, strbArrival] = getstrbFriction(kinFriction, strbInf, risingCst, dwellTime,...
                dwellTimePrev, frictionCoeffPrev, strbFrictionPrev, strbArrivalPrev)
            
            if dwellTime == 0
                strbArrival = strbArrivalPrev;
                if dwellTimePrev ~=0 %breakaway
                    strbFriction = frictionCoeffPrev - kinFriction;
                else
                    strbFriction = strbFrictionPrev;
                end
            else
                if dwellTimePrev == 0
                    strbArrival = frictionCoeffPrev - kinFriction;
                else
                    strbArrival = strbArrivalPrev;
                end
                strbFriction = strbArrival + (strbInf - strbArrival)*exp(-risingCst/dwellTime);
            end
            
        end
        
        function frictionCoeff = getFrictionCoeff(velocityD, kinFriction, strbFriction, strbVelocity)
            
            if velocityD == 0
                frictionCoeff = kinFriction + strbFriction;
            else
                frictionCoeff = kinFriction + strbFriction*exp(-abs(velocityD)/strbVelocity)^2;
            end
            
        end
        
        function psiStar = getPsiStar(Dvar, pressureAngle, leadAngle, frictionCoeff, preload)
            
            reposeAngleEquiv = atan(frictionCoeff/cos(pressureAngle));
            
            downhill = tan(leadAngle - reposeAngleEquiv);
            uphill = tan(leadAngle + reposeAngleEquiv);
            
            if Dvar > 0
                if preload == 0
                    psiStar = downhill;
                else
                    psiStar = [downhill uphill];
                end
            else
                if preload == 0
                    psiStar = uphill;
                else
                    psiStar = [uphill downhill];
                end
            end
            
        end
        
        function equivInertia = getEquivInertia(gamma0, psi0, gamma, psiStar, inElemInertia,...
                mnOutElemInertia, loadInertia, secOutElemInertia)
            
            if secOutElemInertia == 0
                equivInertia = inElemInertia + gamma0*psi0*gamma*psiStar(1)*(mnOutElemInertia + loadInertia);
            else
                equivInertia = inElemInertia + ...
                    gamma0*psi0*gamma*dot(psiStar,[mnOutElemInertia;secOutElemInertia] + [loadInertia;0]);
            end
            
        end
        
        function supportTorque = getSupportTorque(gamma0, psi0, gamma, psiStar, inSupBreak, inSupVisc,...
                outSupBreak, outSupVisc, preload, tendency, velocity)
            
            if velocity == 0
                if preload == 0
                    supportTorque = inSupBreak*sign(tendency) + gamma*psiStar(1)*outSupBreak*sign(tendency);
                else
                    supportTorque = inSupBreak*sign(tendency) + gamma*psiStar(1)*outSupBreak*sign(tendency)...
                        + preload*gamma*(psiStar(2)-psiStar(1));
                end
            else
                if preload == 0
                    supportTorque = inSupBreak*sign(velocity) + inSupVisc*velocity...
                        + gamma*psiStar(1)*outSupBreak*sign(gamma0*psi0*velocity)...
                        + gamma*psiStar(1)*outSupVisc*(gamma0*psi0*velocity);
                else
                    supportTorque = inSupBreak*sign(velocity) + inSupVisc*velocity...
                        + gamma*psiStar(1)*outSupBreak*sign(gamma0*psi0*velocity)...
                        + gamma*psiStar(1)*outSupVisc*(gamma0*psi0*velocity)...
                        + preload*gamma*(psiStar(2)-psiStar(1));
                end
            end
            
        end
        
        function equivTorque = getEquivTorque(actrTorque, payload, gamma, psiStar, supportTorque)
            
            uTorque = actrTorque + gamma*psiStar(1)*payload;
            
            equivTorque = uTorque - supportTorque;
            
        end
        
        function acceleration = getAcceleration(equivTorque, equivInertia, tendency, velocity)
            
            if velocity == 0
                if sign(tendency) ~= sign(equivTorque)
                    acceleration = 0;
                else
                    acceleration = equivTorque/equivInertia;
                end
            else
                acceleration = equivTorque/equivInertia;
            end
            
        end
        
        function torque = getTorque(gamma, psiStar, payload, equivInertia, supportTorque, acceleration,...
                velocity)
                        
            if velocity == 0
                torque = 0;
            else
                torque = -gamma*psiStar(1)*payload + equivInertia*acceleration + supportTorque;                
            end
            
        end
        
        
    end
    
    
    methods
        %% Constructor
        
        function set.geometry(drive, geometry)
            
            parameters = [geometry.lead, geometry.leadAngle, geometry.threadAngle, geometry.inElemDia,...
                geometry.inElemEffDia, geometry.outElemDia, geometry.outElemEffDia, geometry.reductionRatio];
            
            if ~isempty(parameters(parameters < 0))
                error('Geometry error: invalid parameters provided')
            end
            
            drive.geometry.threadAngle = geometry.threadAngle;
            
            if ~ismember(geometry.type, [1 2 3])
                error('Unknown transmission type')
            else
                drive.geometry.type = geometry.type;
            end
            
            if drive.geometry.type ~= 3
                LSDgeometry = Transmission.setLSDgeometry(geometry.lead, geometry.leadAngle,...
                    geometry.inElemDia, geometry.reductionRatio);
                
                drive.geometry.lead = LSDgeometry(1);
                drive.geometry.leadAngle = LSDgeometry(2);
                drive.geometry.inElemDia = LSDgeometry(3);
                drive.geometry.reductionRatio = LSDgeometry(4);
            else
                WDgeometry = Transmission.setWDgeometry(geometry.lead, geometry.leadAngle,...
                    geometry.inElemDia, geometry.outElemDia, geometry.reductionRatio);
                
                drive.geometry.lead = WDgeometry(1);
                drive.geometry.leadAngle = WDgeometry(2);
                drive.geometry.inElemDia = WDgeometry(3);
                drive.geometry.outElemDia = WDgeometry(4);
                drive.geometry.reductionRatio = WDgeometry(5);
            end
            
            if geometry.inElemEffDia == 0 || geometry.inElemEffDia > drive.geometry.inElemDia
                drive.geometry.inElemEffDia = drive.geometry.inElemDia;
            else
                drive.geometry.inElemEffDia = geometry.inElemEffDia;
            end
            
            if geometry.outElemEffDia == 0 || geometry.outElemEffDia > drive.geometry.outElemDia
                drive.geometry.outElemEffDia = drive.geometry.outElemDia;
            else
                drive.geometry.outElemEffDia = geometry.outElemEffDia;
            end
            
            drive.geometry.pressureAngle = atan(tan(drive.geometry.threadAngle/2)*cos(drive.geometry.leadAngle));
            
            drive.geometry = struct('type', drive.geometry.type, 'lead', drive.geometry.lead,...
                'leadAngle', drive.geometry.leadAngle, 'inElemDia', drive.geometry.inElemDia,...
                'inElemEffDia', drive.geometry.inElemEffDia, 'outElemDia', drive.geometry.outElemDia,...
                'outElemEffDia', drive.geometry.outElemEffDia, 'reductionRatio', drive.geometry.reductionRatio,...
                'threadAngle', drive.geometry.threadAngle, 'pressureAngle', drive.geometry.pressureAngle);
            
        end
        
        function set.preload(drive, preload)
                        
            if preload.springForce < 0
                error('Preload error: invalid parameters provided')
            else
                drive.preload.springForce = preload.springForce;
            end
            
        end
        
        function set.friction(drive, friction)
            
            parameters = [friction.kinFriction, friction.strbVelocity, friction.risingCst...
                friction.memoryCst, friction.inSupBreak, friction.inSupVisc, friction.outSupBreak,...
                friction.outSupVisc];
            
            if ~isempty(parameters(parameters < 0))
                error('Friction error: invalid parameters provided')
            else
                drive.friction.inSupBreak = friction.inSupBreak;
                drive.friction.inSupVisc = friction.inSupVisc;
                drive.friction.outSupBreak = friction.outSupBreak;
                drive.friction.outSupVisc = friction.outSupVisc;
                drive.friction.kinFriction = friction.kinFriction;
                drive.friction.strbInf = friction.strbInf;
                drive.friction.strbVelocity = friction.strbVelocity;
                drive.friction.risingCst = friction.risingCst;
                drive.friction.memoryCst = friction.memoryCst;
            end
            
            drive.friction = struct('inSupBreak', drive.friction.inSupBreak,...
                'inSupVisc', drive.friction.inSupVisc, 'outSupBreak', drive.friction.outSupBreak,...
                'outSupVisc', drive.friction.outSupVisc, 'kinFriction', drive.friction.kinFriction,...
                'strbInf', drive.friction.strbInf, 'strbVelocity', drive.friction.strbVelocity,...
                'risingCst', drive.friction.risingCst, 'memoryCst', drive.friction.memoryCst);
            
        end
        
        function set.inertia(drive, inertia)
            
            parameters = [inertia.inElemInertia, inertia.mnOutElemInertia, inertia.secOutElemInertia];
            
            if ~isempty(parameters(parameters < 0))
                error('Inertia error: invalid parameters provided')
            else
                drive.inertia.inElemInertia = inertia.inElemInertia;
                drive.inertia.mnOutElemInertia = inertia.mnOutElemInertia;
                drive.inertia.secOutElemInertia = inertia.secOutElemInertia;
            end
            
            drive.inertia = struct('inElemInertia', drive.inertia.inElemInertia, 'mnOutElemInertia',...
                drive.inertia.mnOutElemInertia, 'secOutElemInertia', drive.inertia.secOutElemInertia);
            
        end
        
        function set.actuation(drive, actuation)
            
            parameters = [actuation.nominalTorque, actuation.torqueCst, actuation.encoderCounts];
            
            if ~isempty(parameters(parameters < 0))
                error('Actuation error: invalid parameters provided')
            else
                drive.actuation.nominalTorque = actuation.nominalTorque;
                drive.actuation.torqueCst = actuation.torqueCst;
                drive.actuation.encoderCounts = actuation.encoderCounts;
            end
            
            drive.actuation = struct('nominalTorque', drive.actuation.nominalTorque, 'torqueCst',...
                drive.actuation.torqueCst, 'encoderCounts', drive.actuation.encoderCounts);
            
        end
        
    end
    
    
    methods(Access = protected)
        %% Dynamics
        function setupImpl(drive)
            
            drive.gamma0 = Transmission.getGamma(drive.geometry.inElemDia, drive.geometry.outElemDia);
            
            drive.gamma = Transmission.getGamma(drive.geometry.inElemEffDia, drive.geometry.outElemEffDia);
            
            drive.psi0 = Transmission.getPsi0(drive.geometry.leadAngle);
            
        end
        
        function [torque, acceleration, frictionCoeff, strbFriction, strbArrival] = ...
                stepImpl(drive, acceleration, velocity, velocityD, actrTorque, loadInertia, payload,...
                dwellTime, dwellTimePrev, frictionCoeffPrev, strbFrictionPrev, strbArrivalPrev)
            
            
            velocity(abs(velocity) <= drive.zeroTolerance) = 0;
            velocityD(abs(velocityD) <= drive.zeroTolerance) = 0;
            
            Transmission.nonCompression(drive.preload.springForce, payload, drive.gamma0, drive.psi0,...
                acceleration, velocity, drive.inertia.mnOutElemInertia, loadInertia,...
                drive.inertia.secOutElemInertia, drive.friction.outSupBreak, drive.friction.outSupVisc)
            
            [strbFriction, strbArrival] = Transmission.getstrbFriction(drive.friction.kinFriction,...
                drive.friction.strbInf, drive.friction.risingCst, dwellTime, dwellTimePrev,...
                frictionCoeffPrev, strbFrictionPrev, strbArrivalPrev);
            
            frictionCoeff = Transmission.getFrictionCoeff(velocityD, drive.friction.kinFriction,...
                strbFriction, drive.friction.strbVelocity);
            
            tendency = actrTorque + drive.gamma0*drive.psi0*payload;
            
            Dvar = Transmission.getDvar(drive.preload.springForce, payload, drive.gamma0, drive.psi0,...
                tendency, acceleration, velocity, drive.inertia.mnOutElemInertia, loadInertia,...
                drive.friction.outSupBreak, drive.friction.outSupVisc);
            
            psiStar = Transmission.getPsiStar(Dvar, drive.geometry.pressureAngle, drive.geometry.leadAngle,...
                frictionCoeff, drive.preload.springForce);
            
            equivInertia = Transmission.getEquivInertia(drive.gamma0, drive.psi0, drive.gamma, psiStar,...
                drive.inertia.inElemInertia, drive.inertia.mnOutElemInertia, loadInertia,...
                drive.inertia.secOutElemInertia);
            
            supportTorque = Transmission.getSupportTorque(drive.gamma0, drive.psi0, drive.gamma, psiStar,...
                drive.friction.inSupBreak, drive.friction.inSupVisc, drive.friction.outSupBreak,...
                drive.friction.outSupVisc, drive.preload.springForce, tendency, velocity);
            
            equivTorque = Transmission.getEquivTorque(actrTorque, payload, drive.gamma, psiStar,...
                supportTorque);
            
            torque = Transmission.getTorque(drive.gamma, psiStar, payload, equivInertia, supportTorque,...
                acceleration, velocity);
            
            acceleration = Transmission.getAcceleration(equivTorque, equivInertia, tendency, velocity);
            
                        
        end
        
    end
    
    
end