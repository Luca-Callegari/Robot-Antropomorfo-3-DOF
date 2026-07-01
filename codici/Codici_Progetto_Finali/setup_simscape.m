%% =========================================================================
%  SETUP_SIMSCAPE  -  Prepara i dati per il modello Simscape Multibody
%  =========================================================================
%  La dinamica e' calcolata automaticamente da Simscape.
%  Questo script calcola solo q_ref (set-point costante dalla IK)
%  e i guadagni PID da passare a Simulink.
% =========================================================================

clear; clc; close all;

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
%  [4]  TARGET DA TASTIERA
%  =========================================================================
fprintf('Workspace: sfera raggio [%.1f, %.1f] centrata in (0, 0, %.1f)\n', ...
        abs(l2-l3), l2+l3, d1);

centro     = [0; 0; d1];
r_max      = l2 + l3;
r_min      = abs(l2 - l3);
XYZtarget  = zeros(3, 1);
target_ok  = false;

while ~target_ok
    XYZtarget(1) = input('  x = ');
    XYZtarget(2) = input('  y = ');
    XYZtarget(3) = input('  z = ');

    dist = norm(XYZtarget - centro);

    if dist < r_min
        fprintf('  [ERRORE] Target troppo vicino alla base: distanza %.4f m < r_min %.4f m. Riprova.\n', dist, r_min);
    elseif dist > r_max
        fprintf('  [ERRORE] Target fuori dal workspace: distanza %.4f m > r_max %.4f m. Riprova.\n', dist, r_max);
    else
        fprintf('  [OK] Target accettato. Distanza dal centro: %.4f m\n', dist);
        target_ok = true;
    end
end

%% =========================================================================
%  [5]  CINEMATICA INVERSA - q_ref costante
%  =========================================================================
nCoords = length(f);
soglia  = 0.4;
tf      = 20;
qinit   = [0; 0; 0];   % configurazione iniziale non singolare

[~, q_ik] = ik_iterativa(qinit, XYZtarget, f_func, jac_func, ...
                          nDOF, nCoords, tf, soglia, 0.01, 1e-2, 10000);

% Set-point costante: configurazione finale della IK
q_star = q_ik(end, :)';

% Verifica convergenza
q_cell   = num2cell(q_star);
ee_final = f_func(q_cell{:});
fprintf('q_star = [%.4f, %.4f, %.4f] rad\n', q_star);
fprintf('Errore IK: %.6f m\n', norm(ee_final - XYZtarget));

% Costruisce segnale costante per il blocco "From Workspace" di Simulink
t_ref   = (0:0.01:tf)';
nPoints = length(t_ref);

q_ref_simulink.time               = t_ref;
q_ref_simulink.signals.values     = repmat(q_star', nPoints, 1);
q_ref_simulink.signals.dimensions = nDOF;

%% =========================================================================
%  [6]  CONDIZIONI INIZIALI E GUADAGNI PID
%  =========================================================================
q0_pd  = qinit;
dq0_pd = zeros(nDOF, 1);

k  = 5;
kp = k;   
kd = k+1;   
ki = 2;   

%% =========================================================================
%  [7]  ESPORTA VARIABILI NEL WORKSPACE per Simscape
%  =========================================================================
assignin('base', 'tf',             tf);
assignin('base', 'kp',             kp);
assignin('base', 'kd',             kd);
assignin('base', 'ki',             ki);
assignin('base', 'q0_pd',          q0_pd);
assignin('base', 'dq0_pd',         dq0_pd);
assignin('base', 'q_ref_simulink', q_ref_simulink);

fprintf('\nVariabili esportate: tf, kp, kd, ki, q0_pd, dq0_pd, q_ref_simulink\n');
fprintf('Avvia la simulazione Simscape.\n');

open_system('SimulazioneRobotSimscape');