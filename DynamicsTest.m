%% Dyanmics Test
clc; clear; close all;

%% Init
x_initial = 0; 
y_inital = 0; 
spd = 50;
th0 = deg2rad(-30);
bank = deg2rad(0);

bMax = deg2rad(20);
brMax = deg2rad(5);

uav = uavDynamics();
uav = uav.initState(x_initial, y_inital, spd, th0, bank, bMax, brMax);

%% Clothoid 
f = @(x) 750+x*0;

g=9.81;
kmax  = g/spd^2*tan(bMax);  
dkmax = (g/spd^3)*brMax*sec(bMax)^2;
k_i = g/spd^2*tan(bank);

opt = Clothoid3ArcOptim(f, x_initial, y_inital, th0, k_i, kmax, dkmax, spd);
[X,fval,output,S] =opt.ClothoidOptimOutputs();

%% Loop
figure;
pid = PID_control(15, 0, 75);
pid.set_point = th0; pid.saturationLimit = brMax;
pid.Integrator_max = brMax; pid.Integrator_min=-brMax;

dt = .01;
t = 0; 
h = th0; h_t = pid.set_point;
x = x_initial; y = y_inital;
offset = 50; 

i = 2;
clothoid = true; last_plot = 0;
while true
    t(i) = t(i-1) + dt;
    
    if x(i-1) < X(1)
        [x_temp, y_temp, s, t_s, iflag, dst ] = S.closestPoint(x(i-1),y(i-1));               
        dist = sqrt((x(i-1)-x_temp)^2 + (y(i-1)-y_temp)^2);
        
        if s+offset < fval && x(i-1) < X(1)
            [x_t(i), y_t(i)] = S.evaluate(s+offset);
            pid.set_point = atan2(y_t(i)-y(i-1),x_t(i)-x(i-1));

        else
            x_t(i) = x(i-1)+offset;
            y_t(i) = f(x_t(i));
            pid.set_point = atan2(y_t(i)-y(i-1),x_t(i)-x(i-1));
        end
    else
        x_t(i) = x(i-1)+offset;
        y_t(i) = f(x_t(i));
        pid.set_point = atan2(y_t(i)-y(i-1),x_t(i)-x(i-1));
    end
    
%     Dynamics % Plotting
      
    pid = pid.Update(h(i-1),t(i));
    u(i) = pid.GetNewControlValue();
      
    uav = uav.updateState(u(i),dt);
    [x(i), y(i), h(i)] = uav.getState(); 
    h_t(i) = pid.set_point;
    b(i) = uav.phi;
    
    
    if t(i) - last_plot > .5
        plot(x,y, 'k:',x(i),y(i),'r*');
        hold on;
        fplot(f, 'b-','LineWidth',2); plot(X(1),f(X(1)),'*g');
        hold off;
        axis equal; grid on;
        pause(dt/1000);
        last_plot = t(i);
    end

    if t(i) < 90
        i=i+1;
    else
        disp('Ending Simulation');
        break
    end
end

%% Plots
arc = linspace(0,fval);
[x_s, y_s] = S.evaluate(arc);

figure('DefaultAxesFontSize',14); hold on;
fplot(f, 'b-','LineWidth',2); 
plot(X(1),f(X(1)),'*g','MarkerSize',10);
plot(x_s,y_s,'r-','LineWidth',3);
plot(x,y,'k--','LineWidth',2);
quiver(x_initial,y_inital,cos(th0),sin(th0), 'AutoScaleFactor', 400,'MaxHeadSize',50,'LineWidth',2,'Color',	[0 1 0],'Marker','o','MarkerSize',6)
legend('Target Path', 'Rejoin point','Return Path', 'Simulated UAV Path', 'Starting Position/Heading'); 
axis equal; grid;
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

% bank angle
figure;
plot(t,rad2deg(b),'b');
xlabel('Time (s)'); ylabel('Bank Angle'); grid;

%% XTE Plot
for i=1:length(x)
    if x(i) < X(1)
        [x_temp, y_temp, s, t_s, iflag, dst ] = S.closestPoint(x(i),y(i));
        xte(i) = dst;
    else
        fun = @(xx) xte_solve(xx,f,x(i),y(i));
        x_temp = fminsearch(fun,x(i));
        y_temp = f(x_temp);
        xte(i) = hypot(x(i)-x_temp,y(i)-y_temp);
    end
end
cum_xte = cumtrapz(t,xte);

figure('DefaultAxesFontSize',14);
subplot(211);
plot(t,xte,'LineWidth',2); grid;
xlabel('Time (s)'); ylabel('\newline XTE  (m)');
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xlim([0 max(t)]);
subplot(212);
plot(t,cum_xte,'LineWidth',2); grid;
xlabel('Time (s)'); ylabel('\newline Cumulative XTE (m)'); 
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xlim([0 max(t)]);

%% XTE Solver
function fun = xte_solve(x1,f,x0,y0)
    y1 = f(x1);
    L = hypot(x0-x1,y0-y1);
    fun = abs(L);
end