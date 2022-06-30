% Class Funtions for Three Segment Clothoid Optimization
% Requires clothoid library from: https://github.com/ebertolazzi/Clothoids

classdef Clothoid3ArcOptim
    properties
        % Initial Conditions
        x0
        y0
        th0
        k0
        speed
        
        % Target
        f
        func
        df
        d2f
        kf
        
        
        % Constraints
        kmax
        dkmax
        
        % Optimizer Outputs
        x_result
        fval
        OUTPUT
        flag
        
    end
    methods
        function [x_out,L,out,S] = ClothoidOptimOutputs(obj)
            
            x_out = obj.x_result;
            L = obj.fval;
            out = obj.OUTPUT;
            S = obj.ClothoidCompute(x_out(1),x_out(2),x_out(3));
        end
        
        function obj = Clothoid3ArcOptim(f_path,X0,Y0,theta0,kappa0,Kmax,dKmax,spd)
            lb = [];
            ub = [];
            
            obj.kmax = Kmax;
            obj.dkmax = dKmax;
            
            obj.f = f_path;
            obj.x0 = X0;
            obj.y0 = Y0;
            obj.th0 = rangeSymm(theta0);
            obj.k0 = kappa0;
            obj.speed = spd;
            
            syms xx
            obj.func = obj.f(xx);
            obj.df = diff(obj.func,xx);
            obj.d2f = diff(obj.func,xx,2);
            obj.kf = abs(obj.d2f) ./ (1 + obj.df.^2).^(3/2); 
            obj.kf = matlabFunction(obj.kf);
            obj.df = matlabFunction(obj.df);
            
            A = []; b = []; Aeq = []; beq = [];
            fcost = @(x) obj.ClothoidWrapper(x);
            nonlcon = @(x) obj.ClothoidConstraint(x);
            opts = optimoptions('fmincon','Display','off','Algorithm','sqp','OptimalityTolerance',1e-6,'ConstraintTolerance',1e-6,'MaxIterations',6000,'MaxFunctionEvaluations',8000,'FiniteDifferenceType','central','ScaleProblem',true);
            [~, ~, thf] = obj.endCon(0);
            dtheta = rangeSymm(obj.th0 - rangeSymm(thf));
            if dtheta > deg2rad(90) && dtheta < deg2rad(290)
                [rg, rgA] = obj.rejoin_guess();

                %--------
                x0a = [rgA, obj.kmax/obj.dkmax, obj.kmax/obj.dkmax];
                try
                    [xA,fvalA,exitflagA,outA] = fmincon(fcost,x0a,A,b,Aeq,beq,lb,ub,nonlcon,opts);
                catch 
                    exitflagA = -inf;
                end

                %--------
                x0b = [rg, obj.kmax/obj.dkmax, obj.kmax/obj.dkmax];
                try 
                    [xB,fvalB,exitflagB,outB] = fmincon(fcost,x0b,A,b,Aeq,beq,lb,ub,nonlcon,opts);
                catch
                    exitflagB = -inf; outB = NaN;
                end
                %-------
                if exitflagA >0 && exitflagB >0
                    if fvalA <= fvalB
                        obj.x_result = xA; obj.fval = fvalA; obj.OUTPUT = outA;
                    else
                        obj.x_result = xB; obj.fval = fvalB; obj.OUTPUT = outB;
                    end
                elseif exitflagA >0
                    obj.x_result = xA; obj.fval = fvalA; obj.OUTPUT = outA;
                elseif exitflagB >0
                    obj.x_result = xB; obj.fval = fvalB; obj.OUTPUT = outB;
                elseif exitflagA >=0 && exitflagB >=0
                    warning("WARNING: Exceeded max iterations")
                    if fvalA <= fvalB
                        obj.x_result = xA; obj.fval = fvalA; obj.OUTPUT = outA;
                    else
                        obj.x_result = xB; obj.fval = fvalB; obj.OUTPUT = outB;
                    end
                else
                    obj.x_result = [0 0 0]; obj.fval = NaN; obj.OUTPUT = outB;
                    warning('Failed to converge');
                end
            else
                
                rg = obj.rejoin_guess();
                
                x0b = [rg, obj.kmax/obj.dkmax, obj.kmax/obj.dkmax];
                try 
                    [xB,fvalB,flag,outB] = fmincon(fcost,x0b,A,b,Aeq,beq,lb,ub,nonlcon,opts);
                    obj.x_result = xB; obj.fval=fvalB; obj.OUTPUT = outB;
                if flag <=0
                    warning('Failed to converge?');
                end
                catch 
                    obj.x_result = [0,0,0]; obj.fval = NaN; obj.OUTPUT = NaN;
                end
            end
            
            
        end
        
        function [rg, rgA] = rejoin_guess(obj)
            % Interception Point Guess function must be based on target path and vehilce capabilites and guess function changed to meet those constraints

            % Aimed Forwards
            rg = obj.x0 + 1500;
            
            % Aimed Backwards
            rgA = obj.x0 - 50;
            
        end
        
        function cost = ClothoidWrapper(obj,x)
            S = obj.ClothoidCompute(x(1),x(2),x(3));
            SS0 = S.get(1);
            SS1 = S.get(2);
            SS2 = S.get(3);
            L = SS0.length() + SS1.length() + SS2.length();
            cost = abs(L);
        end
        
        function [c, ceq] = ClothoidConstraint(obj,x)    
            S = obj.ClothoidCompute(x(1),x(2),x(3));

            SS0 = S.get(1); SS1 = S.get(2); SS2 = S.get(3);
            L = SS0.length() + SS1.length() + SS2.length();
            [ ~, ~, kappa ] = S.getSTK();
            
            k1 = abs(kappa(2));
            k2 = abs(kappa(3));
            
            dk1=abs(SS0.dkappa());
            dk2=abs(SS1.dkappa());
            dk3=abs(SS2.dkappa());
            
            % Constraints ----------------------------------------
            c(1) = (dk1 - obj.dkmax);
            c(2) = (dk2 - obj.dkmax);
            c(3) = (dk3 - obj.dkmax);
            c(4) = (k1 - obj.kmax);
            c(5) = (k2 - obj.kmax);
            
            
            % Bounds on max arc length
            cutTH = (cos((SS0.thetaEnd()-SS0.thetaBegin())^4/(32*pi^3)))^3;
            maxS = cutTH*L/3;
            c(6) =  (x(2) - maxS);
            
            cutTH = (cos((SS2.thetaEnd()-SS2.thetaBegin())^4/(32*pi^3)))^3;
            maxS = cutTH*L/3;
            c(7) =  (x(3) - maxS);
            
            % min arc length
            minL = .01;
            c(8) =  (minL-x(2));
            c(9) =  (minL-x(3));
            
            ceq = [];
        end
        
        function [S, i] = ClothoidCompute(obj,x1,s0,s1)
                [y1, k1, th1] = obj.endCon(x1);

                S = ClothoidList();
                i = S.build_3arcG2fixed(...
                            s0,obj.x0,obj.y0,obj.th0,obj.k0,...
                            s1,x1,y1,th1,k1 ...
                            );
        end
        
        
        function [y1, k1, th1] = endCon(obj,x1)
            % f ---> function of y = f(x)
            y1 = obj.f(x1);

            if nargin(obj.kf)>0
                k1 = obj.kf(x1);
            else 
                k1 = obj.kf();
            end
            
            if nargin(obj.df) >0
                th1 = atan2(obj.df(x1),1);
            else
                th1 = atan2(obj.df(),1);
            end
        end
        %end of functions
    end
    
end