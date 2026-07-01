%% =========================================================================
%  SETUP_SIMULINK  -  Prepara i dati per il modello Simulink (dinamica MATLAB)
%  =========================================================================
%  Genera le matrici M, C, G ottimizzate e il q_ref costante dalla IK.
% =========================================================================

clear; close all;

%% =========================================================================
%  [1]  DEFINIZIONE ROBOT
%  =========================================================================
% Variabili reali del robot
d1 = 0.065;  l2 = 0.125;  l3 = 0.086;
m1 = 0.119;    m2 = 0.065;  m3 = 0.101;

% Variabili di prova
% d1 = 0.5;  l2 = 1;  l3 = 0.5;
% m1 = 2;    m2 = 1;  m3 = 0.5;

%        theta   d    alpha    a
DH = [   0,      d1,  pi/2,   0;
         0,      0,   0,      l2;
         0,      0,   0,      l3  ];

nDOF = size(DH, 1);

params.nDOF = nDOF;
params.g    = 9.81;
params.l1   = 0;
params.l2   = l2;
params.l3   = l3;
params.m1   = m1;
params.m2   = m2;
params.m3   = m3;

%% =========================================================================
%  [2]  CINEMATICA DIRETTA
%  =========================================================================
[f, q_sym] = computeKinematics(DH);
disp(vpa(f,3))
%% =========================================================================
%  [3]  JACOBIANA
%  =========================================================================
jac = computeJacobian(f, q_sym);

%% =========================================================================
%  [4]  ENERGIE E EULERO-LAGRANGE
%  =========================================================================
[T, U] = computeEnergies(DH, params);
EL     = eulerLagrange(T, U, q_sym);

fprintf('--- M(q) ---\n'); disp(vpa(EL.M,2));
fprintf('--- C(q,dq) ---\n'); disp(vpa(EL.C_vec,2));
fprintf('--- G(q) ---\n'); disp(vpa(EL.G,2));

%% =========================================================================
%  [5]  GENERAZIONE FILE .m OTTIMIZZATI per Simulink
%  =========================================================================
all_vars = [q_sym; EL.dq];
q_list   = num2cell(q_sym);
  
% L'opzione 'File' è facoltativa, senza questa opzione creo una handle
% function, con questa opzione creo un file
matlabFunction(EL.M,     'Vars', {all_vars}, 'File', 'robot_M_generated');
matlabFunction(EL.C_vec, 'Vars', {all_vars}, 'File', 'robot_C_generated');
matlabFunction(EL.G,     'Vars', {all_vars}, 'File', 'robot_G_generated');
matlabFunction(f,         'Vars', q_list,    'File', 'robot_f_generated');

fprintf('File .m ottimizzati generati.\n');


%% =========================================================================
%  [6]  TARGET DA TASTIERA
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
%  [7]  CINEMATICA INVERSA - q_ref costante
%  =========================================================================
nCoords  = length(f);
soglia   = 0.4;
tf       = 20;
qinit    = [0; 0; 0];

f_func   = matlabFunction(f,   'Vars', q_list);
jac_func = matlabFunction(jac, 'Vars', q_list);

[~, q_ik] = ik_iterativa(qinit, XYZtarget, f_func, jac_func, ...
                          nDOF, nCoords, tf, soglia, 0.001, 1e-5, 50000);

% Set-point costante: ultima configurazione della IK
q_star = q_ik(end, :)';

% Verifica convergenza
q_cell   = num2cell(q_star);
ee_final = f_func(q_cell{:});
fprintf('q_star = [%.4f, %.4f, %.4f] rad\n', q_star);
fprintf('Errore IK: %.6f m\n', norm(ee_final - XYZtarget));

% Segnale costante per il blocco "From Workspace"
t_ref   = (0:0.01:tf)';
nPoints = length(t_ref);

q_ref_simulink.time               = t_ref;
q_ref_simulink.signals.values     = repmat(q_star', nPoints, 1);
q_ref_simulink.signals.dimensions = nDOF;

%% =========================================================================
%  [8]  CONDIZIONI INIZIALI E GUADAGNI PID
%  =========================================================================
q0_pd  = qinit;
dq0_pd = zeros(nDOF, 1);

% Nel caso di masse reali (nell'ordine dei grammi) questo è un controllo
% troppo forte, adatto per robot pesanti, devo dimensionarlo con un fattore
% di scala.
% k  = 5;
% kp = k^2;   
% kd = k;   
% ki = k;   

% M reale: max(diag) = 8.93e-3, ||M|| = 1.03e-2
M_scale = 9e-3;
k = 10;           

kp = k^2 * M_scale;     
kd = 2*k * M_scale;   
ki = kp * 3; % alto per compensare gravità su z             

%% =========================================================================
%  [9]  ESPORTA VARIABILI NEL WORKSPACE per Simulink
%  =========================================================================
assignin('base', 'nDOF',           nDOF);
assignin('base', 'tf',             tf);
assignin('base', 'kp',             kp);
assignin('base', 'kd',             kd);
assignin('base', 'ki',             ki);
assignin('base', 'q0_pd',          q0_pd);
assignin('base', 'dq0_pd',         dq0_pd);
assignin('base', 'XYZtarget',      XYZtarget);
assignin('base', 'q_ref_simulink', q_ref_simulink);

fprintf('\nVariabili esportate. Avvia SimulazioneRobot.slx\n');
open_system('SimulazioneRobot');