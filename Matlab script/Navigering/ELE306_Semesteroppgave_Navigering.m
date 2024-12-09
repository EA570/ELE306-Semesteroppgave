
%% Dette skriptet implementerer gruppe 2 sin navigerings og kontroll strategi for semesteroppgave i ELE306.
%%Både Lattice planlegger med fin og grov oppløsning er inkludert her(Ref
%%rapport
%% Ved testing anbefales det å kommentere ut linje 12 til 18, da dette griddet tar tid å genrere.
%% OccupancyGrid.mat ligger i Github Repository
clear;
close all;

%Laster inn griddet, dette er lagt i GitHub
load occupancyGrid.mat;

%Plotting av occupancy grid. Rødt er opptatt område, hvit er ledig.
figure; 
latticeGrid = Lattice(occupancyGrid);
latticeGrid.plot();
yaxis(size(occupancyGrid,1));
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
title('Occupancy grid');

%Lag lattice griddet med 1 meter sviingradius
disp('Genererer Lattice grid...');
description1 = 'Lattice grid fin oppløsning';
gridFactor = 10; %10 celler = 1 meter
inflationFactor = 10;
gridRoot = [10+inflationFactor 10+inflationFactor 0];
latticeGridMax = ELE306_Generate_Lattice_Grid(occupancyGrid,gridFactor,inflationFactor,gridRoot,description1);
disp('Lattice grid generert!');

%Lag lattice griddet med grovere oppløsning som beskrevet i rapport
disp('Genererer grovere Lattice grid...'); 
description2 = 'Lattice grid grov oppløsning';
gridFactorCoarse = 20;
inflationFactorCoarse = 10;
gridRootCoarse = [20+inflationFactorCoarse 20+inflationFactorCoarse 0];
latticeGridCoarse = ELE306_Generate_Lattice_Grid(occupancyGrid,gridFactorCoarse,inflationFactorCoarse,gridRootCoarse,description2);
disp('Lattice grid generert!');

%Lager veipunktene
init1 = [gridRootCoarse(1,1,1)+gridFactorCoarse gridRootCoarse(1,1,1)+gridFactorCoarse 0];
goal1 = [gridRootCoarse(1,1,1)+22*gridFactorCoarse gridRootCoarse(1,1,1)+4*gridFactorCoarse pi/2];
init2 = goal1;
goal2 = [gridRootCoarse(1,1,1)+2*gridFactorCoarse gridRootCoarse(1,1,1)+5*gridFactorCoarse pi];
init3 = goal2;
goal3 = [gridRootCoarse(1,1,1)+18*gridFactorCoarse gridRootCoarse(1,1,1)+8*gridFactorCoarse pi/2];
init4 = goal3;
goal4 = [gridRootCoarse(1,1,1)+10*gridFactorCoarse gridRootCoarse(1,1,1)+10*gridFactorCoarse pi];
init5 = goal4;
goal5 = [gridRootCoarse(1,1,1)+1*gridFactorCoarse gridRootCoarse(1,1,1)+12*gridFactorCoarse pi];

%For å loope rundt området er siste veipunkt startplassering
init6 = goal5;
goal6 = init1;

disp('Plotter lattice planleggere'); 
figure;
curve1 = latticeGridCoarse.query(init1,goal1);
latticeGridCoarse.plot(); 
title('Veipunkt 1');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

figure;
curve2 = latticeGridCoarse.query(init2,goal2);
latticeGridCoarse.plot(); 
title('Veipunkt 2');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

figure;
curve3 = latticeGridCoarse.query(init3,goal3);
latticeGridCoarse.plot(); 
title('Veipunkt 3');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

figure;
curve4 = latticeGridCoarse.query(init4,goal4);
latticeGridCoarse.plot(); 
title('Veipunkt 4');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

figure;
curve5 = latticeGridCoarse.query(init5,goal5);
latticeGridCoarse.plot();
title('Veipunkt 5');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

figure;
curve6 = latticeGridCoarse.query(init6,goal6);
latticeGridCoarse.plot(); 
title('Veipunkt 6');
xlabel('Grid Celler(10 Celler = 1 meter)');
ylabel('Grid Celler(10 Celler = 1 meter)');
yaxis(size(occupancyGrid,1));

% Kurven til simuleringen bruker ikke theta vinkel, fjerner derfor denne
% kolonnen:
curveTraj1 = curve1(:, 1:2);  
curveTraj2 = curve2(:, 1:2);  
curveTraj3 = curve3(:, 1:2);  
curveTraj4 = curve4(:, 1:2);  
curveTraj5 = curve5(:, 1:2);  
curveTraj6 = curve6(:, 1:2);  

%Slår sammen navigasjonsbanen mellom veipunkt for å få en komplett kurve
%som angir navigasjon for søk:
curveTrajTotal = vertcat(curveTraj1, curveTraj2, curveTraj3, curveTraj4, curveTraj5, curveTraj6);
% Plotter den fullstendige banen
figure;
plot(curveTrajTotal(:,1), curveTrajTotal(:,2), 'k-', 'LineWidth', 1.5);
ylim([0 300]); 
xlim([min(curveTrajTotal(:,1)) - 10, max(curveTrajTotal(:,1)) + 10]); % Adjust X-axis limits if needed
axis manual; 

%Pga problem med simulering fjernes siste verdi i kurven for simulering
% curveTrajTotal = curveTrajTotal(1:end-1, :);

%Implementerer bug2 algoritme for navigering mot objekt som skal plukkes
inflate=10;
bugObjekt = Bug2(occupancyGrid, 'inflate', inflate);

initBug2 = [410 130];%%Velger et punkt på banen til Lattice planner
maalBug2 = [456 190];

%Plotter bug 2
disp('Plotter bug2 navigasjon'); 
figure;
bugObjekt.plot()                          
curvePickUp = bugObjekt.query(initBug2,maalBug2,'animate','current');
title('Naviger mot objekt - Med hindring');
yaxis(size(occupancyGrid,1));

% Gjør kurvene og startpunkt tilgjengelig i Workspace for bruk i SimuLink
assignin('base', 'init1', init1);
assignin('base', 'curveTrajTotal', curveTrajTotal);  
assignin('base', 'curvePickUp', curvePickUp);  

%Kjør simulering i SimuLink
sim1 = sim("sl_followTrajectoryLatticeSearch.slx");
sim2 = sim("sl_followTrajectoryBug2Pickup.slx");