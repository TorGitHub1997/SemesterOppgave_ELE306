clear all; close all; clc;

import ETS3.*  %ETS3.* for 3D, ETS2.* for 2D

%% Robotarm DH parameter og simulasjon
qn = [0 0 0 0 0 0];
L1=0.2; L2=0.4; L3=0.4; L4=0.3; L5=0; L6=0.1;

Lr(1) = Link('revolute','d',L1,'a',0,'alpha',pi/2); % Base
Lr(2) = Link('revolute','d',0,'a',L2,'alpha',0);    % Shoulder
Lr(3) = Link('revolute','d',0,'a',L3,'alpha',0);    % Elbow
Lr(4) = Link('revolute','d',0,'a',L4,'alpha',0);  %Arm
Lr(5) = Link('revolute','d',0,'a',L5,'alpha',pi/2,'offset',pi/2);  %Wrist 
Lr(6) = Link('revolute','d',L6,'a',0,'alpha',0,'offset',-pi/2);  % Wrist Spin

robotarm = SerialLink(Lr,'name', 'Robot-arm')
forwardK = robotarm.fkine(qn);
robotarm.teach;

%% Equivalence of the forward kinematic solution

%Rx rotasjon rundt x-akse
Rx = [1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]
%Ry rotasjon rund y-akse
Ry = [cos(pi/2) 0 sin(pi/2); 0 1 0; -sin(pi/2) 0 cos(pi/2)]

a1 = [0 0 L1]'; a2 = [L2 0 0]'; a3 = [L3 0 0]';
a4 = [L4 0 0]'; a5 = [L5 0 0]'; a6 = [L6 0 0]';

T0 = eye(4)          
T1 = T0*[Rx a1;0 0 0 1]         
T2 = T1*[eye(3) a2;0 0 0 1] %eye(3) --> ingen rotasjon
T3 = T2*[eye(3) a3;0 0 0 1]
T4 = T3*[eye(3) a4;0 0 0 1]
T5 = T4*[eye(3) a5;0 0 0 1]
T6 = T5*[Ry a6;0 0 0 1]

T_FK_peterCorke = robotarm.fkine(qn)
T_FK = T6

%% Differential kinematics 
jointAngles = [0, pi/9, -2*pi/9, -7*pi/18 , 0, 0];  
jointVelocity = [-1 1 3 2 0 0]'; 
jacobianMatrix = robotarm.jacob0(jointAngles);
endEffectorVelocity = jacobianMatrix * jointVelocity;

linearVelocity = endEffectorVelocity(1:3);
angularVelcity = endEffectorVelocity(4:6);
jacobianKinematic = robotarm.fkine(jointAngles);

%% Display Final Configuration
disp('Joint Angles:');
disp(jointAngles);
disp('Joint Velocities:');
disp(jointVelocity);
disp('End-Effector Velocities:');
disp('Linear Velocity:');
disp(linearVelocity);
disp('Angular Velocity:');
disp(angularVelcity);

%% Plot med gitte q-vinkler
q1=0; q2=pi/6; q3=-pi/6; q4=-pi/4; q5=-pi/4; q6=0;
figure('Name','Transformasjons Mapping: Base --> End-effector RobotarmPlot')
robotarm.plot([q1 q2 q3 q4 q5 q6])

T_EF_robotarm = robotarm.fkine([q1, q2, q3, q4, q5, q6]) %forward kinematic av gitte vinkler

%% Plot med qi-vinkler 
qi = robotarm.ikine(T_EF_robotarm) %% inverse kinematic
figure('Name','Transformasjons Mapping: Base --> End-effector RobotarmPlot med qi-verdier')
robotarm.plot(qi);
T_EF_robotarmQi = robotarm.fkine(qi);

% Ser at vinklene ikke er helt like, men pose på end effector er lik,

%% Motion Planning
q_start = [0, 2*pi/3, -2*pi/3, -pi/2, 0, 0];  %start pose
q_object = [-pi/6, pi/9, -2*pi/9, -7*pi/18 , 0, 0];  %object pose
q_prerelease = [pi, 2*pi/3, -2*pi/3, -pi/2, 0, 0]; %prerelease pose
q_release = [pi, pi/3, -11*pi/18,-2*pi/9, 0, 0] ; %release pose

T_start = robotarm.fkine(q_start)
T_object = robotarm.fkine(q_object)
T_prerelease = robotarm.fkine(q_prerelease)
T_release = robotarm.fkine(q_release)

t = linspace(0, 5, 50)  % Time vector

% Joint space
Jtraj1 = jtraj(q_start, q_object, t);
Jtraj2 = jtraj(q_object, q_start, t);
Jtraj3 = jtraj(q_start, q_prerelease, t);
Jtraj4 = jtraj(q_prerelease, q_release, t);
Jtraj5 = jtraj(q_release, q_prerelease, t);
Jtraj6 = jtraj(q_prerelease, q_start, t);


%% Visualize Motion Planning Joint Space
figure('name','Motion Planning Animation Joint Space')
zlim([-1.5,1.5]);
xlim([-1.5,1.5]);
ylim([-1.5,1.5]);
hold on
robotarm.plot(Jtraj1, 'trail', 'r-');  % Visualize the trajectory1
robotarm.plot(Jtraj2, 'trail', 'r-');  % Visualize the trajectory2
robotarm.plot(Jtraj3, 'trail', 'r-');  % Visualize the trajectory3
robotarm.plot(Jtraj4, 'trail', 'r-');  % Visualize the trajectory4
robotarm.plot(Jtraj5, 'trail', 'r-');  % Visualize the trajectory5
robotarm.plot(Jtraj6, 'trail', 'r-');  % Visualize the trajectory6
hold off


%% Position graphs for motionplanner Joint-space
subplot(2,3,1);
plot(t,Jtraj1(:,1)*180/pi,'r-',t,Jtraj1(:,2)*180/pi,'m-',t,Jtraj1(:,3)*180/pi,'g-',t,Jtraj1(:,4)*180/pi,'b-',t,Jtraj1(:,5)*180/pi,'k-',t,Jtraj1(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 1')
legend('q1','q2','q3','q4','q5','q6');
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on;

subplot(2,3,2);
plot(t,Jtraj2(:,1)*180/pi,'r-',t,Jtraj2(:,2)*180/pi,'m-',t,Jtraj2(:,3)*180/pi,'g-',t,Jtraj2(:,4)*180/pi,'b-',t,Jtraj2(:,5)*180/pi,"k-",t,Jtraj2(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 2')
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on

subplot(2,3,3);
plot(t,Jtraj3(:,1)*180/pi,'r-',t,Jtraj3(:,2)*180/pi,'m-',t,Jtraj3(:,3)*180/pi,'g-',t,Jtraj3(:,4)*180/pi,'b-',t,Jtraj3(:,5)*180/pi,'k-',t,Jtraj3(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 3')
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on

subplot(2,3,4);
plot(t,Jtraj4(:,1)*180/pi,'r-',t,Jtraj4(:,2)*180/pi,'m-',t,Jtraj4(:,3)*180/pi,'g-',t,Jtraj4(:,4)*180/pi,'b-',t,Jtraj4(:,5)*180/pi,'k-',t,Jtraj4(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 4')
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on

subplot(2,3,5);
plot(t,Jtraj5(:,1)*180/pi,'r-',t,Jtraj5(:,2)*180/pi,'m-',t,Jtraj5(:,3)*180/pi,'g-',t,Jtraj5(:,4)*180/pi,'b-',t,Jtraj5(:,5)*180/pi,'k-',t,Jtraj5(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 5')
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on

subplot(2,3,6);
plot(t,Jtraj6(:,1)*180/pi,'r-',t,Jtraj6(:,2)*180/pi,'m-',t,Jtraj6(:,3)*180/pi,'g-',t,Jtraj6(:,4)*180/pi,'b-',t,Jtraj6(:,5)*180/pi,'k-',t,Jtraj6(:,6)*180/pi,'c--','LineWidth',1.5)
title('Joint positions, Trajectory 6')
legend('q1','q2','q3','q4','q5','q6')
ylim([-200,200])
xlabel("Time [s]")
ylabel("Degrees [deg]")
grid on

%% RigidBody simulation
%  form på matrise [a alpha d theta]
dhparams = [0      pi/2    0.2       0; 
            0.4       0      0       0; 
            0.4       0      0       0; 
            0.3       0      0       0;
            0      pi/2      0    pi/2;
            0         0    0.1   -pi/2];

robot = rigidBodyTree;        
bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i==1 % Add first Body to base
        addBody(robot,bodies{i},"base")
    else
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

figure(Name="Model robotarm")
show(robot)  % plot of robot-arm
xlim([-0.3,1.5])
ylim([-0.5,0.8])
zlim([-0.5,0.8])
%Interactive simulation of robot-arm
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
xlim([-0.3,1.5])
ylim([-1,1])
zlim([-1,1])
