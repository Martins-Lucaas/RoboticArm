%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ur_with_covvi_combined_scaledurdf.m
% Importa o UR-e + COVVI Hand com escala corrigida editando o URDF da mão
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%% 0) Diretórios principais (ajuste conforme sua estrutura) 
root      = 'C:\Users\lucas\OneDrive\Documentos\Faculdade\RoboticArm\TCC\matlab';
armURDF   = fullfile(root, 'ur_e_description','urdf','universalUR16e.urdf');   % troque por UR30e
handURDF  = fullfile(root, 'covvi_urdf','urdf','linear_covvi_hand_right.urdf');

%% 1) Gera um URDF temporário da mão com escala (mm→m) no próprio XML
txt      = fileread(handURDF);
% Insere scale="0.001 0.001 0.001" em cada tag <mesh ... />
txt2     = regexprep( ...
    txt, ...
    '<mesh\s+filename="([^"]+)"\s*/>', ...
    '<mesh filename="$1" scale="0.001 0.001 0.001"/>' ...
);
scaledURDF = fullfile(root,'covvi_urdf','urdf','linear_covvi_hand_scaled.urdf');
fid = fopen(scaledURDF,'w'); 
fprintf(fid, '%s', txt2); 
fclose(fid);

%% 2) Torna as pastas de malha visíveis ao importador
addpath(fullfile(root,'ur_e_description','meshes'));
addpath(fullfile(root,'covvi_urdf'));

%% 3) Importa braço e mão (agora com escala correta)
arm  = importrobot(armURDF,  'DataFormat','row');
hand = importrobot(scaledURDF,'DataFormat','row');

%% 4) Cria mount fixo no flange (tool0) do braço e acopla a mão
Tflange2hand = trvec2tform([0 0 0.063]) * ...      % offset 63 mm
               axang2tform([0 1 0 pi])  * ...      % vira palma para frente
               axang2tform([1 0 0 pi]);            % dedos para cima

mount      = rigidBody('hand_mount');
jMount     = rigidBodyJoint('hand_mount_joint','fixed');
setFixedTransform(jMount, Tflange2hand);
mount.Joint = jMount;

addBody(arm, mount, 'tool0');
addSubtree(arm, 'hand_mount', hand, 'ReplaceBase', true);

%% 5) Visualiza o conjunto completo
figure('Color','w','Name','UR-e + COVVI Hand (escala corrigida)');
show(arm, arm.homeConfiguration, 'Frames','off','Visuals','on');
view(135,25); axis equal;
title('UR-e com COVVI Hand (malhas da mão escaladas no URDF)');
