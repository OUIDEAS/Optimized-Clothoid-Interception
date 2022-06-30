%% Clothoid Optimization Contour Plot
close all; clc; clear; close all;
NN = 30;

%% Intial Conditions 
x0 = 0; y0 = 0;
theta0 = deg2rad(0);
bMax = deg2rad(20);
brMax = deg2rad(5);
bank = 0;
f= @(x) x*0+750; 

%%  Constraints
spd = 50; g=9.81; 
kmax  = g/spd^2*tan(bMax);  
dkmax = (g/spd^3)*brMax*sec(bMax)^2;
kappa0 = g/spd^2*tan(bank);
RejoinMax = 3500; RejoinMin = 0;

%%
opt = Clothoid3ArcOptim(f,x0,y0,theta0,kappa0,kmax,dkmax,spd);
[x,fval,output,S] = opt.ClothoidOptimOutputs();
SS1 = S.get(1); SS2 = S.get(3);
SMAX = max([x(2) x(3)])*2;

xlist = linspace(RejoinMin,RejoinMax,NN);
SS = ClothoidList();
A = linspace(.05,SMAX,NN);
B = A;
k = 0;
prog = 0;
WBar = waitbar(0,'Computing Clothoids');
for s0 = A
    k = k+1;
    i = 0; 
    for s1 = B
        i = i+1;
        j = 0;
        for x1 = xlist
            j = j+1;
            
            prog = prog + 1;
            waitbar(prog/NN^3,WBar)
            
            [y1, kappa1, theta1] = opt.endCon(x1);
            
            iter = SS.build_3arcG2fixed(...
                 s0,x0,y0,theta0,kappa0,...
                 s1,x1,y1,theta1,kappa1 ...
            );
            SS0 = SS.get(1);
            SS1 = SS.get(2);
            SS2 = SS.get(3);

            [ ~, ~, kappa ] = SS.getSTK();
            Z(k,i,j) = NaN; KMAX(k,i,j) = NaN; K(k,i,j) = NaN; L(k,i,j) = NaN;

            if iter > 0 && SS0.length() > 0 && SS1.length() > 0 && SS2.length() > 0
                KMAX(k,i,j)= max(abs([SS0.dkappa(),SS1.dkappa(),SS2.dkappa()]));
                K(k,i,j) = max(max(abs(kappa)));
                L(k,i,j) = SS0.length() + SS1.length() + SS2.length();
                K1(k,i,j) = abs(kappa(2));
                K2(k,i,j) = abs(kappa(3));
                DK1(k,i,j)= abs(SS0.dkappa());
                DK2(k,i,j)= abs(SS1.dkappa());
                DK3(k,i,j)= abs(SS2.dkappa());
            end
        end
    end
end
close(WBar);

%% Plotting
arc = linspace(0,fval);
[x_s, y_s] = S.evaluate(arc);
figure('DefaultAxesFontSize',14); hold on;
fplot(f, 'k-','LineWidth',2); 
plot(x(1),f(x(1)),'*g','MarkerSize',10);
plot(x_s,y_s,'b:','LineWidth',3);
quiver(x0,y0,cos(theta0),sin(theta0), 'AutoScaleFactor', 400,'MaxHeadSize',50,'LineWidth',2,'Color',	[0 1 0])
legend('Target Path', 'Rejoin point','Return Path', 'Starting Position/Heading'); 
axis equal; grid;
ax = gca;
outerpos = ax.OuterPosition; ti = ax.TightInset; 
left = outerpos(1) + ti(1); bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3); ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xlim([0 x(1)+100]);


xslice = [.05];
yslice = [.05];
zslice = [min(xlist)];

figure('DefaultAxesFontSize',14);
hold on;
p1 = patch(isosurface(A,B,xlist,K, kmax));
p1.FaceVertexAlphaData = 0.2; 
p1.FaceAlpha = 'flat' ;
p1.FaceColor = 'r';
p1.EdgeAlpha = .4;
p1 = patch(isosurface(A,B,xlist,KMAX, dkmax));
p1.FaceVertexAlphaData = 0.2; 
p1.FaceAlpha = 'flat' ;
p1.FaceColor = 'g';
p1.EdgeAlpha = .4;
plot3(x(3),x(2),x(1),'w.','MarkerSize',30);
contourslice(A, B, xlist, L, xslice, yslice, zslice,50); C=caxis; caxis([C(1),C(2)])
legend('Max Curvature','Max Curvature Rate','Optimal Point')
xlabel('S_0'); ylabel('S_1'); zlabel('Rejoin Point');
cb = colorbar;                                  % create and label the colorbar
cb.Label.String = 'Total Clothoid Length';

%----------------------
figure('DefaultAxesFontSize',14);
hold on;
p1 = patch(isosurface(A,B,xlist,K1, kmax));
p1.FaceVertexAlphaData = 0.2; 
p1.FaceAlpha = 'flat'; 
p1.FaceColor = 'r';
p1 = patch(isosurface(A,B,xlist,K2, kmax));
p1.FaceVertexAlphaData = 0.2;
p1.FaceAlpha = 'flat';    
p1.FaceColor = 'y';

p1 = patch(isosurface(A,B,xlist,DK1, dkmax));
p1.FaceVertexAlphaData = 0.2;  
p1.FaceAlpha = 'flat' ;
p1.FaceColor = 'g';
p1 = patch(isosurface(A,B,xlist,DK2, dkmax));
p1.FaceVertexAlphaData = 0.2;  
p1.FaceAlpha = 'flat' ; 
p1.FaceColor = 'b';
p1 = patch(isosurface(A,B,xlist,DK3, dkmax));
p1.FaceVertexAlphaData = 0.2;   
p1.FaceAlpha = 'flat' ;          
p1.FaceColor = 'c';
plot3(x(3),x(2),x(1),'w.','MarkerSize',30);

contourslice(A, B, xlist, L, xslice, yslice, zslice,50); C=caxis; caxis([C(1),C(2)]);
xlabel('S_0'); ylabel('S_1'); zlabel('Rejoin Point');
cb = colorbar;
cb.Label.String = 'Total Clothoid Length';
legend('Max Curvature 1','Max Curvature 2','Max Curvature Rate 1', 'Max Curvature Rate 2', 'Max Curvature Rate 3','Optimal Point')
