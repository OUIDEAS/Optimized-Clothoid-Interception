classdef PID_control
    properties
        
        Kp = 0;
        Ki =0;
        Kd =0;
        
        Integrator = 0;
        Integrator_max = inf;
        Integrator_min = -inf;
        
        Derivator = 0;
        power
        error
        
        P_value
        I_value
        D_value
        
        last_error = 0;
        last_value = 0;
        set_point = 0;
        
        saturationLimit = inf;
        prev_t = 0;
        
        out = 0;
    end
    methods      
        function obj = PID_control(Kp, Ki, Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end
        
        function newControlValue = GetNewControlValue(obj)
            newControlValue = obj.out;
        end
        
        function obj = Update(obj, current_value, current_time)
            dt = (current_time - obj.prev_t);
            obj.prev_t = current_time;
            obj.error = obj.set_point - current_value;

            obj.P_value = obj.Kp * obj.error;
            change = obj.error - obj.last_error;

            obj.I_value = obj.Integrator * obj.Ki * dt;

            obj.D_value = obj.Kd * change / dt;
            obj.Derivator = obj.error;

            obj.Integrator = obj.Integrator + obj.error;

            if obj.Integrator > obj.Integrator_max
                obj.Integrator = obj.Integrator_max;
            elseif obj.Integrator < obj.Integrator_min
                obj.Integrator = obj.Integrator_min;
            end
            obj.last_error = obj.error;
            obj.last_value = current_value;

            obj.out = obj.P_value + obj.I_value + obj.D_value;
            
            if abs(obj.out) > abs(obj.saturationLimit)
                obj.out = sign(obj.out)*obj.saturationLimit;
            end
        end
    end
end