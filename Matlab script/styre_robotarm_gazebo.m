clc; 
clear; 
close all;

% ROS 2-node og publisher
rosNode = ros2node('/matlab_test_node', 76);
jointTrajPub = ros2publisher(rosNode, '/boat_arm_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory');

% Opprett meldingen
jointTrajMsg = ros2message('trajectory_msgs/JointTrajectory');
jointTrajMsg.joint_names = {'arm_base_joint', 'link_1_armbaat_joint', 'link_2_armbaat_joint', 'link_3_armbaat_joint'};



% offset verdier for og matche modell i gazebo med matlab
% vi plusser dei inn til posisjonane som er henta frå simulering_og_baneplanlegging scriptet
q3 = -pi/2;
q4 = -pi/4;

% posisjoner lest av frå simulering_og_baneplanlegging 
posisjoner = [
     0.0,     0.0,    0.0,         0.0;  
     pi,      0.0,    q3,          q4;    
     pi/2,    0.0,    q3,          q4;   
     0.0 ,    0.0,    q3,          q4;   
     0.6202, -0.8207, 0.0245 + q3, 0.0108 + q4;   
     0.6202, -1.4827, 0.3091 + q3, 0.3882 + q4; 
     0.6202, -0.8207, 0.0245 + q3, 0.0108 + q4;
     0.0 ,    0.0,    q3,          q4;
     pi/2,    0.0,    q3,          q4;
     pi,      -pi/4,  q3,          q4;
     pi,      0.0,    q3,          q4;


];


% Tid i sekunder for hver bevegelse
tidsinterval = 2;

% Send hver posisjon separat

for i = 1:size(posisjoner, 1)
    %opretter ny melding til ros 
    point = ros2message('trajectory_msgs/JointTrajectoryPoint');

    % lese verdier frå matrisen med posisjoner 
    point.positions = posisjoner(i, :);

    point.time_from_start.sec = int32(tidsinterval); 
    

    % Fjern unødvendige felter
    point.velocities = [];
    point.accelerations = [];
    point.effort = [];

    % Legg punktet til meldingen
    jointTrajMsg.points = point;

    % Send meldingen
    send(jointTrajPub, jointTrajMsg);

    % Pause for å gi roboten tid til å nå posisjonen
    pause(tidsinterval + 1);
end

disp('Alle posisjoner er blit sendt.');