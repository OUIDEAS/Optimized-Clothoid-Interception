%% Angle Change Test
clc; clear; close all;

%% Intial Conditions 
x0 = 0; y0 = 0;
th0 = deg2rad(0);
bMax = deg2rad(30);
brMax = deg2rad(5);
bank = 0;
f = @(x) x*0+1000;

%%  Constraints
spd = 50; g=9.81; 
kmax  = g/spd^2*tan(bMax);  
dkmax = (g/spd^3)*brMax*sec(bMax)^2;
k0 = g/spd^2*tan(bank);

%% Optimization
N = 30;
th0 = linspace(0,2*pi-.01,N);
for i = 1:length(th0)
    th0(i) = rangeSymm(th0(i));
end

%% Loop
figure('DefaultAxesFontSize',14); hold on;
fplot(f, 'LineWidth',2)
plot(x0,y0,'g*');

WBar = waitbar(0,'Computing Clothoids');
prog = 0;
for i = 1:length(th0)
    prog = prog + 1;
    waitbar(prog/N,WBar)
    
    clear optim s x fval output arc; pause(.01);
    optim = Clothoid3ArcOptim(f,x0,y0,th0(i),k0,kmax,dkmax, spd);
    [x, fval, output, S] = optim.ClothoidOptimOutputs();
    SS0 = S.get(1); SS1 = S.get(2); SS2 = S.get(3);

    Length(i) = S.length();
    arc = linspace(0,S.length);
    [x_s, y_s] = S.evaluate(arc);
    plot(x_s,y_s,'-r');
end
close(WBar);

%% Plotting -----
plot(x0,y0,'g*');
xlabel('X (m)'); ylabel('Y (m)');
legend('Target Path', 'Starting Position', 'Return Paths');
grid; axis equal;
ax = gca; 
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure('DefaultAxesFontSize',14); 
plot(rad2deg(th0),Length,'LineWidth',2); grid; 
xlabel('Initial Heading \psi_0 (deg)'); ylabel('Clothoid Length (m)'); 
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xlim([0 360]);
