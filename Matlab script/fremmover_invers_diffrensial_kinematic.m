clear;
close;



%lengder på leddene i robot arm 
a1 = 0.15;       
a2 = 0.65;      
a3 = 0.55;      
a4 = 0.2; 

L(1) = Link('revolute', 'd', a1, 'a', 0, 'alpha', pi/2);       
L(2) = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0, 'offset', -pi/2);
L(4) = Link('revolute', 'd', 0, 'a', a4, 'alpha', 0, 'offset', -pi/4);


% Bygg modell av robotarmen 
baatrobotarm = SerialLink(L, 'name', '4-DoF Robot arm');

% skriver ut DH tabell til robotarmen
baatrobotarm.display();

% denne har vi brukt til og finne rekkevidde på robotarmen 
baatrobotarm.teach();

%% for og tesete ut fremmover kinematic og samanligne med transformasjonsmatriser 


%sette inn verdier for og teste fremovekinematic, kan teste med forskjellige vinkler 
a = 0;        
b = 0;
c = 0;
d = 0;

q_ledd = [a, b, c, d];

%utskrift av transformasjonsmatrise/fremmover kinematic med corke toolboks 
baatrobotarm_fremoverkinematic = baatrobotarm.fkine(q_ledd);

disp("transformasjonsmatrise til fremover kinematic")
disp(baatrobotarm_fremoverkinematic.T);


baatrobotarm.teach(q_ledd);

%% diffrensial kinematic til roboten 

% putte inn ulike verdier får vinkler for og beregne kraft i akktuatorane. 
q_start = deg2rad([0, -90, 90, 46.6]);

% vekta til plastikken som båten skal plukke opp 
massen_til_plastikk = 4;
massen_til_endeefektor = 1;
gravitasjonskonstant = 9.81;
totalmasse = massen_til_endeefektor + massen_til_plastikk;
F = totalmasse * gravitasjonskonstant;  

% kraftvektor 
W0 = [0; 0; F; 0; 0; 0];

% Beregn Jacobian-matrisen 
J0 = baatrobotarm.jacob0(q_start);
disp('jacobian matrisen')
disp(J0),


% Beregn kor stor kraft som kvart ledd må ha i leddene
Q = J0' * W0; % jacobian matrise er transponert 

disp('krefter i kvart ledd:');
disp(Q);

% tegne kraftepiloiden for og finne ut kor mykje kvart ledd jobber i ulike
% posisjoner 

figure;
baatrobotarm.teach(q_start,'callback', @(r,q) r.fellipse(q), 'workspace', [-3 3 -3 3 -3 3]);

%% invers kinematic 
% blir brukt i andre script her er kunn for og teste og sammanligne med
% fremmmoverkinematic 


innvers_kinematic = baatrobotarm.ikine(baatrobotarm_fremoverkinematic, 'mask', [1 1 1 1 0 0]);

% Utskrift av resultat
disp('inverskinematic:');
disp(innvers_kinematic);