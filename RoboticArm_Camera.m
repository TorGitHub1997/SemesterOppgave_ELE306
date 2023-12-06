clear all; close all; clc;

import ETS3.*  %ETS3.* for 3D, ETS2.* for 2D

qn = [7.2 -7.2 -18 -46.8 0 0];
L1=0.2; L2=0.4; L3=0.4; L4=0.3; L5=0.1;

Lr(1) = Link('revolute','d',L1,'a',0,'alpha',pi/2); % Base
Lr(2) = Link('revolute','d',0,'a',L2,'alpha',0);    % Shoulder
Lr(3) = Link('revolute','d',0,'a',L3,'alpha',0);    % Elbow
Lr(4) = Link('revolute','d',0,'a',L4,'alpha',0);  %Arm
Lr(5) = Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',pi/2);  %Wrist 
Lr(6) = Link('revolute','d',L5,'a',0,'alpha',0,'offset',-pi/2);  % Wrist Spin

robotarm = SerialLink(Lr,'name', 'Robotarm');
forwardK = robotarm.fkine(qn);
robotarm.teach;

jointAngles = [-45 20 45 -20 0 0 ];
jointVelocity = [-10 0 0 0 0 0 ]';
jacobianMatrix = robotarm.jacob0(jointAngles);
endEffectorVelocity = jacobianMatrix * jointVelocity;

linearVelocity = endEffectorVelocity(1:3);
angularVelcoity = endEffectorVelocity(4:6);
jacobianKinematic = robotarm.fkine(jointAngles);

forwardK_1 = [ -0.9918   -0.1211    0.0415   0.04413; 
               -0.1280    0.9382   -0.3215    -0.342;
                0   -0.3241   -0.9460    0.4738;
                0         0         0         1];
%Camera 

camera_P = [0.7 0.10 0.3];
T_Camera = transl(camera_P) * rpy2tr(120,10,160);
T_Camera_Base = (forwardK_1) * T_Camera;

C_T_PlasticPose = transl(0.841,0.4,-0.6);
C_T_PlasticRotation = rpy2tr(130, 10,200,'deg');
C_T_Plastic = C_T_PlasticRotation * C_T_PlasticPose;

Base_To_Plastic = T_Camera_Base * C_T_Plastic;

d_Config = robotarm.ikcon(Base_To_Plastic);

c_Config = robotarm.ikcon(forwardK);
motionPlanning = jtraj(c_Config,d_Config,50);

robotarm.plot(motionPlanning);

%T_Object = transl(0.164,-0.675,-0.554) * rpy2tr(160,1,120,'deg');

%config2 = robotarm.ikcon(forwardK);
%config1 = robotarm.ikcon(T_Object);

%motionPlanning = jtraj(config2 ,config1 ,50);

%robotarm.plot(motionPlanning);

%returnPlanning = jtraj(config1 ,config2 ,50);

%robotarm.plot(returnPlanning);

disp('Joint Angles:');
disp(jointAngles);
disp('Joint Velocities:');
disp(jointVelocity);
disp('End-Effector Velocities:');
disp('Linear Velocity:');
disp(linearVelocity);
disp('Angular Velocity:');
disp(angularVelcoity);