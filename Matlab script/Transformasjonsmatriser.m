clc; 
clear all; 
close all;
%transformasjon matriser til robot amrm, kamera og plastikk.


% posisjon på kamera i forhold til båt
x = 0.2; 
y = 0.0;
z = 0.1;

%transformasjonsmatrise fra robotarmbase til kamera, rotert med 15 grader
%ned mot vannet
T_robotarmbase_kamera = [cos(-pi/12),   0,  sin(-pi/12),  x;  
                         0          ,   1,      0,        y;  
                         -sin(-pi/12),  0,  cos(-pi/12),  z;  
                         0,             0,      0,        1];

% viser matrisen
disp('Transformasjonsmatrise fra robotarmbase til kamera:');
disp(T_robotarmbase_kamera);



%transformasjonsmatrise frå kamera til plastikk
% kameraet detekterer plastikk så her vil verdier variere
syms x1 y1 z1;



T_kamera_plastikk =    [1, 0, 0, x1;  
                        0, 1, 0, y1;  
                        0, 0, 1, z1;  
                        0, 0, 0, 1];
% viser matrisen
disp('Transformasjonsmatrise mellom kamera og plastikk:');
disp(T_kamera_plastikk);


% viser matrisen
disp('Transformasjonsmatrise mellom båt og robotarm base:');
disp(T_kamera_plastikk);

%% utrekninger av fremover kinematik/transformasjonsmatrise til robotarm

syms q1 q2 q3 q4

%transformasjonsmatrise til robotarmbase_ledd1
% DH-parametere til ledd 1 
a = 0; 
alpha = pi/2;
d = 0.15;
theta = q1;
%transformasjonsmatrise
T_robotarmbase_ledd1 = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                         0,           sin(alpha),            cos(alpha),            d;
                         0,           0,                     0,                     1];
T_robotarmbase_ledd1 = vpa(T_robotarmbase_ledd1, 3);

% utskrift av matrisen 
disp('Transformasjonsmatrise mellom robotarm base og ledd1:');
disp(T_robotarmbase_ledd1);



%%transformasjonsmatrise til ledd1_ledd2
% DH-parametere til ledd 2 
a = 0.65; 
alpha = 0;
d = 0;
q2_offset = (pi/2);
theta = q2 + q2_offset;
%transformasjonsmatrise
T_ledd1_ledd2 = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                0,           sin(alpha),            cos(alpha),            d;
                0,           0,                     0,                     1];
T_ledd1_ledd2 = vpa(T_ledd1_ledd2, 3);
% utskrift av matrisen 
disp('Transformasjonsmatrise mellom ledd1 og ledd2:');
disp(T_ledd1_ledd2);




%%transformasjonsmatrise til ledd2_ledd3
% DH-parametere til ledd 3 
a = 0.55; 
alpha = 0;
d = 0;
q3_offset = -(pi/2);
theta = q3 + q3_offset;
%transformasjonsmatrise 
T_ledd2_ledd3 = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                0,           sin(alpha),            cos(alpha),            d;
                0,           0,                     0,                     1];
T_ledd2_ledd3 = vpa(T_ledd2_ledd3, 3);
% utskrift av matrisen 
disp('Transformasjonsmatrise mellom ledd2 og ledd3:');
disp(T_ledd2_ledd3);






%%transformasjonsmatrise til ledd3_ledd4
% DH-parametere til ledd 4 
a = 0.2; 
alpha = 0;
d = 0;
q4_offset = -(pi/4);
theta = q4 + q4_offset;
%transformasjonsmatrise 
T_ledd3_ledd4 = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                0,           sin(alpha),            cos(alpha),            d;
                0,           0,                     0,                     1];
T_ledd3_ledd4 = vpa(T_ledd3_ledd4, 3);
% utskrift av matrisen 
disp('Transformasjonsmatrise mellom ledd3 og ledd4:');
disp(T_ledd3_ledd4);



% utrekningar av transformasjons matrise frå base til ende efektor
T_robotarmbase_endeefektor = T_robotarmbase_ledd1 * T_ledd1_ledd2 * T_ledd2_ledd3 * T_ledd3_ledd4;

% utrykket blir veldig langt så bruker simplify til og forenkle det litt
T_robotarmbase_endeefektor = simplify(T_robotarmbase_endeefektor);
T_robotarmbase_endeefektor= vpa(T_robotarmbase_endeefektor,3);
disp('T_robotarmbase_endeefektor = ');
disp(T_robotarmbase_endeefektor);

%% testing av vinkler til fremover kinematicen for og samanlige med robot corks toolboks

% matlab kode for og sette inn verdier  i fremmoverkinematicen, kan brukast
% til og samanlige resultat mellom peter corks tulboks og manuelle
% beregninger.

% kan putte inn ulike verdier for vinklene for og teste under her  
 % = 0; % vinkel til q1       
 % = 0; % vinkel til q2
 % = 0; % vinkel til q3
 % = 0; % vinkel til q4

% skrive ut matrise med numeriske verdier for og sammanligne med corke toolboks 
T_robotarmbase_endeefektor_med_insatte_verdier= subs(T_robotarmbase_endeefektor, [q1 q2 q3 q4], [a b c d]);
T_robotarmbase_endeefektor_med_numeriskverdier = double(T_robotarmbase_endeefektor_med_insatte_verdier);

% utskrift av transformasjons matrise
disp('fremoverkinematic med innsatte verier for vinkler');
disp(T_robotarmbase_endeefektor_med_insatte_verdier);

% utskrift med numeriske verdier 
disp ('transformasjonsmatrise med numeriske verdier')
disp(T_robotarmbase_endeefektor_med_numeriskverdier)

%% transformasjons matriser mellom robotarmbase og plastikk 

% transformasjons matrise fra robotarmbase til plastikk  
T_robotarmbase_plastikk = T_robotarmbase_kamera * T_kamera_plastikk;
T_robotarmbase_plastikk= vpa(T_robotarmbase_plastikk,3);
disp('Transformasjonsmatrise mellom robotarmbase og plastikk:');
disp(T_robotarmbase_plastikk);