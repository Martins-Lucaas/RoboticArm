%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ur30e_covvi_workspace.m  – todas as juntas livres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%% 0)  Pasta-raiz do projeto  (ajuste se necessário)
projectRoot = "C:\Users\lucas\OneDrive\Documentos\Faculdade\RoboticArm\TCC\matlab";
cd(projectRoot);

%% 1)  Arquivos URDF ----------------------------------------------------------
armURDF  = fullfile(projectRoot,"ur_e_description","urdf","universalUR16e.urdf");
handURDF = fullfile(projectRoot,"covvi_urdf","urdf","linear_covvi_hand_right.urdf");

%% 2)  Malhas (MeshPath) ------------------------------------------------------
meshRoots = [ ...
    fullfile(projectRoot,"ur_e_description"), ...
    fullfile(projectRoot,"covvi_urdf") ];

arm  = importrobot(armURDF , "DataFormat","row", "MeshPath",meshRoots);
hand = importrobot(handURDF, "DataFormat","row", "MeshPath",meshRoots);

%% 3)  Cria um "mount" fixo no flange e acopla a COVVI Hand  ------------------
Tflange2hand = trvec2tform([0 0 0.063]);     % deslocamento flange → mão (ajuste)

% 3a) Corpo adaptador entre tool0 e a mão
mount      = rigidBody('covvi_mount');
jMount     = rigidBodyJoint('covvi_mount_joint','fixed');
setFixedTransform(jMount,Tflange2hand);
mount.Joint = jMount;
addBody(arm,mount,'tool0');                  % mount é filho direto do flange

% 3b) Anexa a árvore da COVVI como sub-árvore do mount
addSubtree(arm,'covvi_mount',hand,'ReplaceBase',false);

eeName = 'base';                     % confirme com showdetails(arm)

%% 4)  Solver de IK -----------------------------------------------------------
ik = inverseKinematics('RigidBodyTree',arm);      % objeto IK padrão
ik.SolverParameters.MaxIterations     = 150;      % limite de iterações
ik.SolverParameters.SolutionTolerance = 1e-4;     % (opcional) tolerância

weights = [0.2 0.2 0.2  1 1 1];
qSeed   = arm.homeConfiguration;

%% 5)  Amostragem do workspace ------------------------------------------------
Npts = 4000;   Rmax = 1.3;
reached = nan(Npts,3);  nOK = 0;

rng(0)
for k = 1:Npts
    dir = randn(1,3); dir = dir/norm(dir);
    p   = dir * rand()*Rmax;
    T   = trvec2tform(p);

    [q,info] = ik(eeName,T,weights,qSeed);
    if info.ExitFlag == 1
        nOK = nOK+1;  reached(nOK,:) = p;  qSeed = q;
    end
end
reached = reached(1:nOK,:);

%% 6)  Visualização -----------------------------------------------------------
figure('Color','w','Name','Workspace UR30e + COVVI (todas juntas livres)');
scatter3(reached(:,1),reached(:,2),reached(:,3),10,'filled'); hold on;
show(arm,q,'PreservePlot',false,'Frames','off','Visuals','on');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Pontos alcançáveis: %d / %d amostras', nOK, Npts));
grid on; axis equal; view(135,25);
