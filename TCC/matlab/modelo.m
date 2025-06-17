%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ur_with_covvi_simple.m – braço UR-e + COVVI Hand (sem renomear nada)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%% 0) Caminhos (edite conforme sua pasta)
root = 'C:\Users\lucas\OneDrive\Documentos\Faculdade\RoboticArm\TCC\matlab';

armURDF  = fullfile(root,'ur_e_description','urdf','universalUR16e.urdf');  % troque p/ UR30e
handURDF = fullfile(root,'covvi_urdf','urdf','linear_covvi_hand_right.urdf');

meshRoots = { ...
    char(root,'ur_e_description'), ...
    char(root,'ur_e_description','meshes'), ...
    char(root,'covvi_urdf') };

%% 1) Importa braço e mão
arm  = importrobot(armURDF ,'DataFormat','row','MeshPath',meshRoots);
hand = importrobot(handURDF,'DataFormat','row','MeshPath',meshRoots);

%% 2) Mount fixo entre flange (tool0) e mão
Tflange2hand = trvec2tform([0 0 0.063]) * ...      % offset 63 mm (edite se preciso)
               axang2tform([0 1 0 pi])  * ...      % vira palma para frente
               axang2tform([1 0 0 pi]);            % põe dedos para cima

mount      = rigidBody('hand_mount');
jMount     = rigidBodyJoint('hand_mount_joint','fixed');
setFixedTransform(jMount,Tflange2hand);
mount.Joint = jMount;

addBody(arm,mount,'tool0');                       % mount filho do flange
addSubtree(arm,'hand_mount',hand,'ReplaceBase',true);

%% 3) Mostra conjunto
figure('Color','w','Name','UR-e + COVVI Hand (import direto)');
show(arm,arm.homeConfiguration,'Frames','off','Visuals','on');
view(135,25); axis equal;
title('Braço UR-e com COVVI Hand acoplada');
