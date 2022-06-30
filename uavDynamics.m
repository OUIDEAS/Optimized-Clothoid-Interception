classdef uavDynamics
    % 2D Fixed Wing UAV Vehicle Model
    % Bank Angle and Bank Angle Rate Constraints are included
    properties
        x
        y
        basespd % m/s
        heading % rad
        phi % rad
        
        %Bounds
        bankMax % rad
        bankRateMax %rad/s
    end
    
    methods 
        
        function obj = initState(obj,x_i, y_i, spd_i, heading_i, bankAngle, bankMax, bankRateMax)
            obj.x = x_i;
            obj.y = y_i;
            obj.basespd = spd_i;
            obj.heading = heading_i;
            obj.bankMax = abs(bankMax);
            obj.bankRateMax = abs(bankRateMax);
            obj.phi = bankAngle;
        end
        
        function [X, Y, HEADING] = getState(obj)
            X = obj.x;
            Y = obj.y;
            HEADING = obj.heading;
        end
        
        function obj = updateState(obj, u, dt)
            if abs(u) <= obj.bankRateMax
                bankRate = u;
            else
                bankRate = sign(u) * obj.bankRateMax;
            end
            
            bankAngle = obj.phi + bankRate*dt;
            
            if abs(bankAngle) <= obj.bankMax
                obj.phi = bankAngle;
            else
                obj.phi = sign(bankAngle)*obj.bankMax;
            end
            
            g = 9.81;
            obj.heading = obj.heading + (g/obj.basespd)*tan(obj.phi)*dt;
            
            obj.y = obj.y + obj.basespd*sin(obj.heading)*dt;
            obj.x = obj.x + obj.basespd*cos(obj.heading)*dt;
        end
    end
    
end