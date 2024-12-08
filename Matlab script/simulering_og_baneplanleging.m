clc; 
clear; 
close all;

%lengder på leddene i robot arm 
a1 = 0.15;       
a2 = 0.65;      
a3 = 0.55;      
a4 = 0.2;       

% dh parameterene 
L(1) = Link('revolute', 'd', a1, 'a', 0, 'alpha', pi/2);       
L(2) = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0, 'offset', -pi/2);
L(4) = Link('revolute', 'd', 0, 'a', a4, 'alpha', 0, 'offset', -pi/4);

baatrobotarm = SerialLink(L, 'name', 'robot arm til båt');

% posisjoner til robot arm når den går fra start og skal begynne og plukke 
% dette er faste posisjoner som ikkje er avhengig av kameraet
q_start = [deg2rad(180), 0, 0, 0];
q_mellom_posisjon = [deg2rad(90), 0, 0, 0];
q_mellom_posisjon_2 = [0, 0, 0, 0];
q_doroppoff_plastikk = [deg2rad(180), deg2rad(-45), 0, 0];

% kameraet sender inn informasjon om plasering til plastikk
% kan putte inn ulike verdiar for og teste. 
x_posisjon_plastikk = 0.7;
y_posisjon_plastikk = 0.5;
z_posisjon_plastikk = -0.5;

% ønsket posisjon på endeefektor 
% Ønsket rotasjon for at endeeffektor peiker ned mot overflate 
R_posisjon_endeeffektor = [0  0 -1  0;  
             0  1  0  0;  
            -1  0  0  0;   
             0  0  0  1]; 

% transformasjons matrise som beskriver posisjon til plastikken 
T_plastikk = SE3(x_posisjon_plastikk, y_posisjon_plastikk, z_posisjon_plastikk)  * SE3(R_posisjon_endeeffektor);
%lager også en transformasjonsmatrise med posisjon rett over plastikk,
%denne tar x og y kordinater fra kameraet, men vi holder z verdien konstant
T_over_plastikk = SE3(x_posisjon_plastikk, y_posisjon_plastikk, 0)   * SE3(R_posisjon_endeeffektor);

% beregner vinkler til rotasjonsledd i robotarmen får posisjonen til over_plastikk
q_over_plastikk = baatrobotarm.ikine(T_over_plastikk, 'mask', [1 1 1 0 0 1], 'q0', q_mellom_posisjon_2);



%Lager tidsvektorar til alle punkta 
% kan endrest vist ein vil andre verdier på bevegelse, men vi har brukt likt
% på alle banene 
t1 = [0:0.05:2];
t2 = [0:0.05:2];
t3 = [0:0.05:2];
t4 = [0:0.05:2];
t5 = [0:0.05:2];
t6 = [0:0.05:2];
t7 = [0:0.05:2];
t8 = [0:0.05:2];
t9 = [0:0.05:2];
t10 = [0:0.05:2];

% bevegelser frå start posisjon til posisjon får og plukke opp plastikk
q1 = mtraj(@tpoly, q_start, q_mellom_posisjon, t1);
q2 = mtraj(@tpoly, q_mellom_posisjon, q_mellom_posisjon_2, t2);

% bevegelse til rett over plastikk
q3 = mtraj(@tpoly, q_mellom_posisjon_2, q_over_plastikk, t3);


% Bevegelse ned til plastikk, her bruker vi ctraj for og få ei rett bane
T_bane_ned_til_plastikk = ctraj(T_over_plastikk, T_plastikk, length(t4));
q_bane_ned_til_plastikk = baatrobotarm.ikine(T_bane_ned_til_plastikk, 'mask', [1 1 1 0 0 1], 'q0', q_over_plastikk);

% her lukker gripperen seg
disp('griper lukker seg ');
pause(2);

% Bevegelse opp igjen til posisjon over plastikken, bruker også ctraj her
T_bane_opp_fra_plastikk = ctraj(T_plastikk, T_over_plastikk, length(t5));
q_bane_opp_fra_plastikk = baatrobotarm.ikine(T_bane_opp_fra_plastikk, 'mask', [1 1 1 0 0 1], 'q0', q_bane_ned_til_plastikk(end, :));

% får og komme seg tilbake får og legge fra seg plastikk til lagring.
q6 = mtraj(@tpoly, q_over_plastikk, q_mellom_posisjon_2, t6);

q7 = mtraj(@tpoly, q_mellom_posisjon_2, q_mellom_posisjon, t7);

q8 = mtraj(@tpoly, q_mellom_posisjon, q_doroppoff_plastikk, t8);

% gripperen åpner seg 
disp('gripperen opner for og legge fra seg plastikk');
pause(2);

% tilbake til start posisjon
q9 = mtraj(@tpoly, q_doroppoff_plastikk, q_start, t9);


% heile bana til robot armen 
q_total = [q1; q2; q3; q_bane_ned_til_plastikk; q_bane_opp_fra_plastikk; q6; q7; q8; q9];

% for og skrive ut verdier som vi skal bruke til å simulere i gazebo
disp('Vinkler for posisjon rett over plastikk:');
disp(q_over_plastikk);

disp('Vinkler for posisjon ned til plastikk:');
disp(q_bane_ned_til_plastikk(end, :));

% lage figuren 
figure;
baatrobotarm.plot(q_total);












