%% =========================================================================
%  SETUP_ROBOT_DATA  -  Dati cinematici condivisi tra tutte le fasi
%  =========================================================================
%  Chiamato da fase1_ottimizzazione.m e setup_simscape_feedforward.m
%  Carica nel workspace: DH, nDOF, f_func, jac_func, f, jac, q_sym
% =========================================================================

d1 = 1.5;  l2 = 1;  l3 = 0.5;

%        theta   d    alpha    a
DH = [   0,      d1,  pi/2,   0;
         0,      0,   0,      l2;
         0,      0,   0,      l3  ];

nDOF = size(DH, 1);

fprintf('Calcolo cinematica simbolica...\n');
[f, q_sym] = computeKinematics(DH);
jac        = computeJacobian(f, q_sym);

q_list   = num2cell(q_sym);
f_func   = matlabFunction(f,   'Vars', q_list);
jac_func = matlabFunction(jac, 'Vars', q_list);

fprintf('Setup robot completato: DH, f_func, jac_func pronti.\n');
