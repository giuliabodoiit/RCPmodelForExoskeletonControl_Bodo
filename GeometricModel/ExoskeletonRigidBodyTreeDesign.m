%% 6DOF UPPER LIMB EXOSKELETON PROTOTYPE, KINEMATIC AND DYNAMIC MODEL
clear all; close all; clc;

% Modified Denavith Hartenberg convenction parameters definition
d1=0;
d2=9.2e-3; %[m]
d3=0;
d4=0;
d5=0;

a1=0; a2=0; a3=0; a4=0; a5=0;

alpha1=0;
alpha2=pi/2;
alpha3=-160*pi/180;
alpha4=59*pi/180;
alpha5=pi/3;

mDH1 = [0,pi,d1,0];
mDH2 = [0,-pi/2,d2,pi/2];
mDH3 = [0,alpha3,d3,pi/3];
mDH4 = [0,alpha4,0,-pi/2];
mDH5 = [0,alpha5,0,0];

% mdh parameters for scapular joints
mdhparams_2dof = [0 pi d1 0;
    0 -pi/2 d2 pi/2;
    0 0 0 0];

% mdh parameters for shoulder joints
mydhparams_3dof = [0 alpha3 d3 pi/3;
    0 alpha4 0 -pi/2;
    0 alpha5 0 0;
    0 0 0 1];

% Additional transformation matrixes needed to correctly define the
% kinematc chain
T3= [1.00000	0.0000000000	0.0000000000	0
    0.0000000000	1.00000	0.0000000000	0.19877
    0.0000000000	0.0000000000	1.00000	0.236333
    0 0 0 1];

T56=[0.0000000000	-1.00000	0.0000064421	-0.000529259e-3
    1.00000	0.0000000000	-0.0000000257	-295.612e-3
    0.0000000257	0.0000064421	1.00000	-0.0000074581e-3
    0 0 0 1];

T6ee=[1.00000	0.0000000000	0.0000000000	0.0000000000e-3
    0.0000000000	1.00000	0.0000000000	-150.000e-3
    0.0000000000	0.0000000000	1.00000	80.3560e-3
    0 0 0 1];

% Joint ROM definition
th1_range = [-42*pi/180 43.96*pi/180];
th2_range = [-43*pi/180 17.47*pi/180];
th3_range = [-58.37*pi/180 72.47*pi/180];
th4_range = [-67.105*pi/180 92.53*pi/180];
th5_range = [-60.79*pi/180 110.98*pi/180];
th6_range = [-88.016*pi/180 42.48*pi/180];

% Transformation matrixes between frames
A10=TmDH(mdhparams_2dof(1,:));

TmDH21=TmDH(mdhparams_2dof(2,:));
A21=trvec2tform([0 0 -0.173338])*TmDH21;

TmDH32=TmDH(mydhparams_3dof(1,:));
A32=trvec2tform([0.048059e-3 198.77e-3 236.333e-3])*TmDH32;

A43=TmDH(mydhparams_3dof(2,:));

A54=TmDH(mydhparams_3dof(3,:));

% RIGID BODY TREE design
ExoskeletonModel = rigidBodyTree;
ExoskeletonModel.Gravity=[0 0 -9.81];

% Joint1: scapular protraction
body1 = rigidBody('body1');
body1.CenterOfMass=[5.3227185e-02  3.6472886e+00 -1.0689989e+02]*1e-3; % data from CAD model
body1.Mass=2.7891860e+00 ; % data from CAD model
body1.Inertia=[1.0235641e+04 1.1235393e+04 3.3699993e+03 1.3102424e+03 1.1186417e+01  5.0405441e+00]*1e-6; % data from CAD model
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.PositionLimits = th1_range;
setFixedTransform(jnt1,TmDH(mdhparams_2dof(1,:)));
body1.Joint = jnt1;
basename = ExoskeletonModel.BaseName;
bodybase= rigidBody('base');
addBody(ExoskeletonModel,body1,basename)

% Joint2: scapular elevation
body2 = rigidBody('body2');
body2.CenterOfMass=[5.5350592e+00  9.2636491e+01 -4.6221487e+01]*1e-3;
body2.Mass=2.9144147e+00;
body2.Inertia=[4.3969642e+04 3.0318204e+03 4.3936348e+04 2.3833806e+03 1.7139145e+02  2.2204015e+02]*1e-6;
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.PositionLimits = th2_range;
TmDH21=TmDH(mdhparams_2dof(2,:));
A21=trvec2tform([0 0 -0.173338])*TmDH21;
setFixedTransform(jnt2,A21);
body2.Joint = jnt2;
addBody(ExoskeletonModel,body2,'body1')

% Joint3: shoulder complex
body3 = rigidBody('body3');
body3.CenterOfMass=[1.6105273e-01 -8.3212269e+01  1.8661296e+02]*1e-3;
body3.Mass= 1.6194238e+00;
body3.Inertia=[1.9506140e+04 1.0143277e+04 1.0093016e+04 -9.0554672e+03 1.3309183e+02  7.5954731e+01]*1e-6;
TmDH32=TmDH(mydhparams_3dof(1,:));
A32=trvec2tform([0.048059e-3 198.77e-3 236.333e-3])*TmDH32;
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.PositionLimits = th3_range;
jnt3.HomePosition = 0;
setFixedTransform(jnt3,A32);
body3.Joint = jnt3;
addBody(ExoskeletonModel,body3,'body2')

% Joint4: shoulder complex
body4 = rigidBody('body4');
body4.CenterOfMass=[1.8140287e-01 -6.4501229e+01  1.1086441e+02]*1e-3;
body4.Mass=1.4377519e+00;
body4.Inertia=[8.5577992e+03 2.6666268e+03 6.4631147e+03  -3.3203801e+03 1.3259917e+02 7.3482248e+01]*1e-6;
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.HomePosition = 0;
jnt4.PositionLimits = th4_range;
TmDH43=TmDH(mydhparams_3dof(2,:));
setFixedTransform(jnt4,TmDH43);
body4.Joint = jnt4;
addBody(ExoskeletonModel,body4,'body3')

% Joint5: shoulder complex
body5 = rigidBody('body5');
body5.CenterOfMass=[2.6237418e-01 -1.4256313e+02  9.8918728e+01]*1e-3;
body5.Mass=1.8609367e+00 ;
body5.Inertia=[ 2.8991203e+04  1.6045900e+03   2.8261020e+04 1.3802979e+03 1.6856413e+01  1.6499917e+02 ]*1e-6;
jnt5 = rigidBodyJoint('jnt5','revolute');
TmDH54=TmDH(mydhparams_3dof(3,:));
jnt5.PositionLimits = th5_range;
setFixedTransform(jnt5,TmDH54);
body5.Joint = jnt5;
addBody(ExoskeletonModel,body5,'body4')

% Joint6: elbow flexion
body6 = rigidBody('body6');
body6.CenterOfMass=[ 2.6780979e-01 -5.4690400e+01  6.8766033e+01]*1e-3;
body6.Mass=1.3019553e+00;
body6.Inertia=[ 5.4780511e+03  8.4113267e+02 5.7331244e+03 -5.8999652e+02 2.1188231e+00 2.1188231e+00]*1e-6;
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.PositionLimits = th6_range;
setFixedTransform(jnt6,T56);
body6.Joint = jnt6;
addBody(ExoskeletonModel,body6,'body5')

% End-effector
bodyEndEffector= rigidBody('endeffector');
bodyEndEffector.Mass= 0;
setFixedTransform(bodyEndEffector.Joint,T6ee);
addBody(ExoskeletonModel,bodyEndEffector,'body6')

showdetails(ExoskeletonModel)

% Graphic visualization of the designed robot model
figure(1)
show(ExoskeletonModel,'visuals','on','collision','off');
ylabel('Y [m]'); xlabel('X [m]');zlabel('Z [m]');
xlim([-0.6 0.4])
zlim([-0.2 0.4])
ylim([-0.7 0.3])
title('6DOF exoskeleton prototype rigid-body-tree')
set(gca,'fontname','times','FontSize',10);
