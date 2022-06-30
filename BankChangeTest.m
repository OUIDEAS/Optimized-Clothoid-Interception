%% Bank Angle Change Test
clc; clear; close all;

%% Intial Conditions 
x0 = 0; y0 = 0;
th0 = deg2rad(0);

brMax = deg2rad(5);
bank = 0;

f= @(x) x*0+750; 
%%  Constraints
spd = 50; g=9.81; 

%% Loop Setup
N=30;
bMax = deg2rad(linspace(5,89,N)); 

%% Loop
WBar = waitbar(0,'Computing Clothoids');
prog = 0;

figure('DefaultAxesFontSize',14); hold on;
fplot(f, 'b-','LineWidth',2); 
quiver(x0,y0,cos(th0),sin(th0), 'AutoScaleFactor', 400,'MaxHeadSize',50,'LineWidth',2,'Color',	[0 1 0],'Marker','o','MarkerSize',6)

for j = 1:N
    
    prog = prog + 1;
    waitbar(prog/N,WBar)
    
    kmax  = g/spd^2*tan(bMax(j));
    dkmax = (g/spd^3)*brMax*sec(bMax(j))^2;
    k0 = g/spd^2*tan(bank);
    km(j) = kmax;
        
    optim = 0;
    optim = Clothoid3ArcOptim(f,x0,y0,deg2rad(th0),k0,kmax,dkmax, spd);
    [x, fval, output, S] = optim.ClothoidOptimOutputs();
    
    X_t(j) = x(1);
    X_S0(j) = x(2);
    X_S1(j) = x(3);
    opt(j) = output.firstorderopt;
    Length(j) = fval;
    rg(j) = optim.rejoin_guess();
    
    arc = linspace(0,S.length);
    [x_s, y_s] = S.evaluate(arc);
    plot(x_s,y_s,'-r');
    plot(x(1),f(x(1)),'g*');
end
close(WBar);

%% Plotting -----
xlabel('X (m)'); ylabel('Y (m)');
grid; legend('Target Path', 'Initial Position/Heading', 'Return Paths', 'Rejoin Position');
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure('DefaultAxesFontSize',14);
plot(rad2deg(bMax),X_t,'LineWidth',2); grid; xlim([min(rad2deg(bMax)) max(rad2deg(bMax))]); 
xlabel('Bank Angle \phi limit (deg)'); ylabel('Rejoin Position (m)');
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
