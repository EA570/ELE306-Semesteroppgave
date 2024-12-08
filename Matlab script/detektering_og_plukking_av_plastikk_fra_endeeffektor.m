% FUlLT skript for deteksjon av plast, og transformasjonsmatriser osv
% VEDLEGG
% Samlet MATLAB-skript for robotarm, sensorer og plastikkdeteksjon
% Dette skriptet kombinerer transformasjoner(world, base, sensor), robotkonfigurasjon og simulering av handlinger
% basert på plastikkdeteksjon. Det inkluderer sensorene IMU, GPS og 3D-kameraer.
clear;
clc;
close all;

%%Definer konstanter og robotparametere
% Lengder på leddene i robotarmen (meter)
a1 = 0.15;
a2 = 0.55;
a3 = 0.45;
a4 = 0.2;

% DH-parametere for robotarmen
L(1) = Link('revolute', 'd', a1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', a4, 'alpha', -pi/2);

% Opprett robotmodell
robotArm = SerialLink(L, 'name', 'Robotarm til båt');

% Startposisjon for robotarmen (alle ledd i nøytral posisjon)
q_start = [0, 0, 0, 0];

%%Transformasjonsmatriser for sensorer og basen
% Justerer kameraene for å peke 7 grader nedover for optimal deteksjon av plast.
vinkelNedover = deg2rad(7); % 7 grader i radianer

imuData1 = 45;
imuData2 = 60;
imuData3 = 30; %oppdiktet input i grader.

xGPS = 10;
yGPS = 10;
zGPS = 5; %Oppdiktet GPS input i meter

% Transformasjon for midtkamera
TkameraM = trotz(0) * troty(-vinkelNedover) * transl(0, 0, -0.2);

% Transformasjon for venstre kamera (litt ut mot venstre og nedover)
TkameraL = trotz(deg2rad(-30)) * troty(-vinkelNedover) * transl(-0.2, 0, -0.2);

% Transformasjon for høyre kamera (litt ut mot høyre og nedover)
TkameraR = trotz(deg2rad(30)) * troty(-vinkelNedover) * transl(0.2, 0, -0.2);

% Transformasjon fra robotarmbase til båtsentrum (IMU/GPS plassering)
TRB = transl(0.5, 0.0, 0.3) * trotz(pi/2); % Juster basert på båtens geometri

% Transformasjon fra båt til verdenskoordinater (basert på GPS og IMU)
roll = deg2rad(imuData1); % IMU-data (roll)
pitch = deg2rad(imuData2); % IMU-data (pitch)
yaw = deg2rad(imuData3); % IMU-data (yaw)
Rotasjon_IMU = rpy2r(roll, pitch, yaw);

% GPS-posisjon (x, y, z i verdensramme)
posisjonGPS = [xGPS; yGPS; zGPS];

% Kombinert transformasjon fra båt til verden
TBW = [Rotasjon_IMU, posisjonGPS; 0 0 0 1];

%%Generer fiktive dybdedata for kameraene
% Simuler dybdebilder med tilfeldig genererte punkter i et gitt område
depthImage1 = rand(100, 100) * 10; % Simulert dybdematrise for midtkamera
depthImage2 = rand(100, 100) * 10; % Simulert dybdematrise for venstre kamera
depthImage3 = rand(100, 100) * 10; % Simulert dybdematrise for høyre kamera

% Genererer punktskyer fra dybdedata for hvert kamera
kamera = pointCloud(depthImage1); % Midtkamera
kameraL = pointCloud(depthImage2); % Venstre kamera
kameraR = pointCloud(depthImage3); % Høyre kamera

% Kombinerer dybdedata fra alle kameraer for enkelhets skyld
kombinertKamera = pccat([kamera, kameraL, kameraR]);

% Segmentering av punktkloud for å finne plastikk
labels = pcsegdist(kombinertKamera, 0.05); % Segmenterer med en avstandsterskel på 0.05 meter
plastikkKluster = find(labels == 1); % Antar at kluster 1 tilsvarer plastikk (kan variere)

% Ekstraher punktene som tilsvarer plastikk
plastikkPunkter = kombinertKamera.Location(plastikkKluster, :);

% Beregn tyngdepunktet (centroid) for plastikken
tyngdepunkt = mean(plastikkPunkter, 1); % [x, y, z]-koordinater

% Tildel tyngdepunktet til sensorvariabler
x_sensor = tyngdepunkt(1);
y_sensor = tyngdepunkt(2);
z_sensor = tyngdepunkt(3);

% Oppdater objektPosisjon med funnet posisjon for plastikken
objektPosisjon = [x_sensor; y_sensor; z_sensor; 1];

% Visualisering av plastikkdeteksjonen (valgfritt for feilsøking)
figure;
pcshow(kombinertKamera); % Vis hele punktkloud
hold on;
plot3(tyngdepunkt(1), tyngdepunkt(2), tyngdepunkt(3), 'ro', 'MarkerSize', 10); % Marker plastens posisjon
title('Funnet plastobjekt');

% Transformerer plastens posisjon til robotarmens base
objektPosisjonBase = TkameraM * objektPosisjon;

%%Invers kinematikk og simulering
% Beregner invers kinematikk for å nå plasten
q_plastikk = robotArm.ikine(transl(objektPosisjonBase(1:3)'), 'mask', [1 1 1 0 0 0], 'q0', q_start);

% Simulerer banen til robotarmen for å nå plasten
t = [0:0.05:2]; % Tidsvektor
q_trajectory = mtraj(@tpoly, q_start, q_plastikk, t);

% Visualisererr banen
figure;
robotArm.plot(q_trajectory);

%%Ekstra funksjonalitet: Bevege armen til plastikkposisjon
robotArm.plot(q_plastikk);