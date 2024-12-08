
clc;
clear;

% ROS 2-node og publisher
% Opprett ROS2-node
test_publisher = ros2node('/test_vm_ros', 76);

% opretter en publisher for og sende data til ros/gazebo
cmdPub = ros2publisher(test_publisher, '/custom_cdm_vel', 'geometry_msgs/Twist');

%oprette subscriber, her kan en motta data frå gazebo
odmSub = ros2subscriber(test_publisher, '/custum_odom_vel', 'nav_msgs/Odometry');


% Opprett melding
cmdMsg = ros2message(cmdPub);

% kan endre på hastighet og svingradius ved og sette inn andre verdier 
% køyre frammover 
cmdMsg.linear.x = -1.5; 
cmdMsg.linear.y = 0.0; 
cmdMsg.linear.z = 0.0;  
cmdMsg.angular.x = 0.0; 
cmdMsg.angular.y = 0.0; 
cmdMsg.angular.z = 0.0; 

% sende meldinger til gazebo
disp('Sender bevegelseskommandoer:');
for cnt = 1:5
    send(cmdPub, cmdMsg);
    pause(1);
end


% svinge til venstre
cmdMsg.linear.x = -0.5; 
cmdMsg.linear.y = 0.0;  
cmdMsg.linear.z = 0.0;  
cmdMsg.angular.x = 0.0; 
cmdMsg.angular.y = 0.0; 
cmdMsg.angular.z = -0.3; 

%sende meldinger til gazebo
disp('Sender bevegelseskommandoer:');
for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);
end


% svinge til høgre
cmdMsg.linear.x = -0.5; 
cmdMsg.linear.y = 0.0;  
cmdMsg.linear.z = 0.0;  
cmdMsg.angular.x = 0.0; 
cmdMsg.angular.y = 0.0; 
cmdMsg.angular.z = 0.3; 

%sende meldinger til gazebo
disp('Sender bevegelseskommandoer:');
for cnt = 1:10
    send(cmdPub, cmdMsg);
    pause(1);
end


% rygging 
cmdMsg.linear.x = 0.5; 
cmdMsg.linear.y = 0.0;  
cmdMsg.linear.z = 0.0;  
cmdMsg.angular.x = 0.0; 
cmdMsg.angular.y = 0.0; 
cmdMsg.angular.z = 0.0; 

%sende meldinger til gazebo
disp('Sender bevegelseskommandoer:');
for cnt = 1:5
    send(cmdPub, cmdMsg);
    pause(1);
end


    

disp('Ferdig med å sende meldinger.');