%% Build 4-DoF SSM

% RCM angles
alpha12 = 30*pi/180;
alpha13 = -23*pi/180;

% Create SSM Joints
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.PositionLimits = [-pi/2 pi/2];
jnt1.HomePosition = 0;
tform1 = trvec2tform([0 0 0])*eul2tform([0 0 pi/2])*eul2tform([0 pi/2 0]);
setFixedTransform(jnt1,tform1);
body1 = rigidBody('body1');
body1.Joint = jnt1;
body1.Mass = 0.72278480;
body1.CenterOfMass = [-0.05164633 -0.00378819 -0.20966862];
body1.Inertia = [0.03364043 0.03668596 0.00363679 0.00061944 0.00666273 0.00009970];

jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.PositionLimits = [-pi/2 pi/2];
jnt2.HomePosition = 0;
tform2 = eul2tform([0 alpha12 0]);
setFixedTransform(jnt2,tform2);
body2 = rigidBody('body2');
body2.Joint = jnt2;
body2.Mass = 0.37375585;
body2.CenterOfMass = [0.04654881 0.01954629 -0.14416952];
body2.Inertia = [0.00901814 0.00928267 0.00215331 -0.00091010 -0.00224833 0.00069739];

jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.PositionLimits = [-pi pi];
jnt3.HomePosition = 0;
tform3 = eul2tform([0 -alpha12 0])*eul2tform([0 0 -pi/2+alpha13])*eul2tform([-pi/2 0 0]);
setFixedTransform(jnt3,tform3);
body3 = rigidBody('body3');
body3.Joint = jnt3;
body3.Mass = 0;
body3.CenterOfMass = [0 0 0];
body3.Inertia = [0 0 0 0 0 0];

jnt4 = rigidBodyJoint('jnt4','prismatic');
jnt4.PositionLimits = [-0.1 0.1];
jnt4.HomePosition = 0.0;
tform4 = trvec2tform([0 0 0]);
setFixedTransform(jnt4,tform4);
body4 = rigidBody('body4');
body4.Joint = jnt4;
body4.Mass = 0.09964338;
body4.CenterOfMass = [-0.05202274 0.00001760 0.10978168];
body4.Inertia = [0.00123747 0.00168529 0.00045452 0.00000019 -0.00051857 -0.00000010];

% Create SSM
robot = rigidBodyTree('DataFormat','row');
robot.Gravity = [0 0 -9.81];
base = robot.BaseName;
addBody(robot,body1,base)
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')

%% Visualizations

% showdetails(robot)
% 
% comPos = centerOfMass(robot);
% 
% figure('Name', 'Robot Configuration')
% show(robot);
% hold on
% scatter3(comPos(1),comPos(2),comPos(3),'or')
