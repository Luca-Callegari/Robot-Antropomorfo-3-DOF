%% =========================================================================
%  SETUP_SIMSCAPE_FEEDFORWARD  -  Main per controllo PID con rete neurale
%  =========================================================================
%  PIPELINE COMPLETA:
%
%  FASE 0 (una tantum):
%    1. Esegui fase1_ottimizzazione.m  -> genera dataset_neurale.mat
%    2. Esegui fase2_training_rete.m   -> genera parametri_rete.mat
%
%  FASE 1 (ogni run):
%    3. Esegui questo script           -> calcola q_ref e prepara workspace
%    4. Avvia SimulazioneRobotSimscapeFeedForward.slx
%
%  NEL MODELLO SIMULINK:
%    - Il blocco MATLAB Function "Tuning" legge q in retroazione
%      e calcola kp, kd, ki tramite la rete neurale e cambiano
%      automaticamente in base alla q presa in retroazione
%    - I guadagni vengono passati direttamente al PID_Controller
%
%  PREREQUISITI:
%    - parametri_rete.mat nella stessa cartella
%    - computeKinematics.m, computeJacobian.m, ik_iterativa.m
% =========================================================================

clear; clc; close all;

%% Verifica che i parametri della rete esistano
if ~exist('parametri_rete.mat', 'file')
    error(['parametri_rete.mat non trovato.\n' ...
           'Esegui prima:\n' ...
           '  1. fase1_ottimizzazione.m\n' ...
           '  2. fase2_training_rete.m']);
end
fprintf('parametri_rete.mat trovato.\n');

%% =========================================================================
%  [1]  DEFINIZIONE ROBOT
%  =========================================================================
d1 = 1.5;  l2 = 1;  l3 = 0.5;

%        theta   d    alpha    a
DH = [   0,      d1,  pi/2,   0;
         0,      0,   0,      l2;
         0,      0,   0,      l3  ];

nDOF = size(DH, 1);

%% =========================================================================
%  [2]  CINEMATICA DIRETTA
%  =========================================================================
[f, q_sym] = computeKinematics(DH);

%% =========================================================================
%  [3]  JACOBIANA
%  =========================================================================
jac = computeJacobian(f, q_sym);

q_list   = num2cell(q_sym);
f_func   = matlabFunction(f,   'Vars', q_list);
jac_func = matlabFunction(jac, 'Vars', q_list);

%% =========================================================================
%  [4]  TARGET DA TASTIERA  (con check workspace)
%  =========================================================================
fprintf('\nWorkspace: sfera raggio [%.1f, %.1f] centrata in (0, 0, %.1f)\n', ...
        abs(l2-l3), l2+l3, d1);

centro    = [0; 0; d1];
r_max     = l2 + l3;
r_min     = abs(l2 - l3);
XYZtarget = zeros(3, 1);
target_ok = false;

while ~target_ok
    XYZtarget(1) = input('  x = ');
    XYZtarget(2) = input('  y = ');
    XYZtarget(3) = input('  z = ');

    dist = norm(XYZtarget - centro);

    if dist < r_min
        fprintf('  [ERRORE] Troppo vicino: %.4f m < %.4f m. Riprova.\n', dist, r_min);
    elseif dist > r_max
        fprintf('  [ERRORE] Fuori workspace: %.4f m > %.4f m. Riprova.\n', dist, r_max);
    else
        fprintf('  [OK] Distanza dal centro: %.4f m\n', dist);
        target_ok = true;
    end
end

%% =========================================================================
%  [5]  CINEMATICA INVERSA - q_ref costante
%  =========================================================================
nCoords = length(f);
soglia  = 0.4;
tf      = 10;
qinit   = [0; pi/6; pi/6];

[~, q_ik] = ik_iterativa(qinit, XYZtarget, f_func, jac_func, ...
                          nDOF, nCoords, tf, soglia, 0.01, 1e-3, 10000);

q_star = q_ik(end, :)';

q_cell   = num2cell(q_star);
ee_final = f_func(q_cell{:});
fprintf('q_star = [%.4f, %.4f, %.4f] rad\n', q_star);
fprintf('Errore IK: %.6f m\n', norm(ee_final - XYZtarget));

% Segnale q_ref costante per From Workspace
t_ref   = (0:0.01:tf)';
nPoints = length(t_ref);

q_ref_simulink.time               = t_ref;
q_ref_simulink.signals.values     = repmat(q_star', nPoints, 1);
q_ref_simulink.signals.dimensions = nDOF;

%% =========================================================================
%  [6]  CONDIZIONI INIZIALI
%  =========================================================================
q0_pd  = qinit;
dq0_pd = zeros(nDOF, 1);

%% =========================================================================
%  [7]  ESPORTA VARIABILI NEL WORKSPACE
%  =========================================================================
%  Nota: kp, kd, ki NON vengono esportati — li calcola la rete neurale
%  direttamente in Simulink leggendo q in retroazione.
assignin('base', 'tf',             tf);
assignin('base', 'q0_pd',          q0_pd);
assignin('base', 'dq0_pd',         dq0_pd);
assignin('base', 'q_ref_simulink', q_ref_simulink);

fprintf('\nVariabili esportate: tf, q0_pd, dq0_pd, q_ref_simulink\n');
fprintf('I guadagni kp, kd, ki sono calcolati dalla rete neurale in Simulink.\n');
fprintf('Avvia SimulazioneRobotSimscapeFeedForward.slx\n');

open_system('SimulazioneRobotSimscapeFeedForward');
