%% =========================================================================
%  FASE 1 - GENERAZIONE DATASET  (Il "Cercatore")
%  =========================================================================
%  Per ogni configurazione q casuale nel workspace, trova i guadagni
%  PID ottimali (kp, kd, ki) che minimizzano l'indice ITAE:
%
%    J = integral( t * ||e(t)|| dt )
%
%  usando fminsearch. Salva il dataset in 'dataset_neurale.mat'.
%
%  PRIMA DI LANCIARE:
%    1. Apri SimulazioneRobotSimscape.slx
%    2. Assicurati che kp, kd, ki vengano letti dal workspace
%       (blocchi Constant con nomi kp_sim, kd_sim, ki_sim)
%    3. Assicurati che il To Workspace dell'errore si chiami 'sim_error'
%       con formato 'Array' e salvi un segnale [T x 3]
% =========================================================================

clear; clc;
setup_robot_data;   % carica DH, f_func, jac_func, nDOF

%% Parametri generazione dataset
n_campioni  = 100;        % numero di configurazioni casuali da esplorare
K_start     = [25, 20, 10];  % punto di partenza ottimizzazione [kp, kd, ki]
K_lb        = [1,  1,  0];   % lower bound guadagni
K_ub        = [500, 100, 50]; % upper bound guadagni
nome_modello = "SimulazioneRobotSimscapeFeedForward";


X_input  = zeros(n_campioni, 3);   % input rete: [q1, q2, q3]
Y_target = zeros(n_campioni, 3);   % output rete: [kp_opt, kd_opt, ki_opt]

J_storia = zeros(n_campioni, 1);

opts_fcon = optimoptions('fmincon', ...
    'Display',       'none', ...
    'MaxIterations', 80,    ...
    'OptimalityTolerance', 1e-2, ...
    'StepTolerance',       1e-2);
%%
for i = 1:n_campioni

    % Configurazione q casuale (valori in radianti ragionevoli)
    q_star = [(rand()-0.5)*2*pi;      % q1: [-π, π]   rotazione base
          (rand()-0.5)*pi;         % q2: [-π/2, π/2] spalla
          (rand()-0.5)*pi];        % q3: [-π/2, π/2] gomito

    X_input(i, :) = q_star';

    fprintf('\n--- Campione %d/%d | q=[%.2f, %.2f, %.2f] ---\n', ...
            i, n_campioni, q_star);

    % Punto di partenza con perturbazione casuale per esplorare meglio
    % Esplora uniformemente tutto lo spazio [K_lb, K_ub]
    K_init = K_lb + rand(1,3) .* (K_ub - K_lb);
    

    % Funzione costo ITAE
    cost_fun = @(K) fitness_pid(K, q_star, nome_modello, K_lb, K_ub);

    % Ottimizzazione
    [K_opt, J_min] = fmincon(cost_fun, K_init, ...
                        [], [], [], [], ...   % no vincoli lineari
                        K_lb, K_ub, ...       % bound rispettati nativamente
                        [], opts_fcon);

    % Clamp e salvataggio
    K_opt = min(max(abs(K_opt), K_lb), K_ub);
    Y_target(i, :) = K_opt;

    fprintf('  kp=%.2f  kd=%.2f  ki=%.2f  |  J=%.4f\n', K_opt, J_min);

    J_storia(i) = J_min;
end

% crea un vettore logico di true/false. Per ogni campione, vale true se il 
% suo ITAE è minore di 3 volte la mediana di tutti gli ITAE
campioni_ok = J_storia < median(J_storia) * 3;
fprintf('Campioni validi: %d/%d\n', sum(campioni_ok), n_campioni);

X_input  = X_input(campioni_ok, :);
Y_target = Y_target(campioni_ok, :);

save('dataset_neurale.mat', 'X_input', 'Y_target');
fprintf('\nDataset salvato in dataset_neurale.mat (%d campioni)\n', n_campioni);

% =========================================================================
function J = fitness_pid(K, q_star, nome_modello, K_lb, K_ub)
% Calcola l'indice ITAE per una data combinazione di guadagni K=[kp,kd,ki]
% simulando il modello Simulink con il target q_star

    % Clamp guadagni positivi
    K = min(max(abs(K), K_lb), K_ub);

    % 2. Crea l'oggetto di configurazione simulazione
    simIn = Simulink.SimulationInput(nome_modello);


    % Passa le variabili DIRETTAMENTE al modello
    % Qui usiamo i nomi esatti che i tuoi blocchi Constant cercano (kp, kd, ki)
    simIn = simIn.setVariable('kp_sim', K(1));
    simIn = simIn.setVariable('kd_sim', K(2));
    simIn = simIn.setVariable('ki_sim', K(3));

    % Costruisce q_ref costante per questo target
    tf_sim = 2;
    t_ref  = (0:0.1:tf_sim)';
    q_ref_sim.time               = t_ref;
    q_ref_sim.signals.values     = repmat(q_star', length(t_ref), 1);
    q_ref_sim.signals.dimensions = 3;

    simIn = simIn.setVariable('q_ref_simulink', q_ref_sim);
    simIn = simIn.setVariable('tf', tf_sim);
    simIn = simIn.setVariable('q0_pd', [0; pi/6; pi/6]);
    simIn = simIn.setVariable('dq0_pd', zeros(3,1));

    try
        %Lancia la simulazione usando l'oggetto simIn
        simOut = sim(simIn);

        t = simOut.tout;

        % 'sim_error' deve essere il nome del To Workspace collegato
        % all'uscita del blocco Sum (errore e = q_ref - q)
        % Formato: Array, dimensioni [T x 3]
        e = simOut.get('sim_error');

        if isempty(e) || isempty(t)
            J = 1e10;
            return;
        end

        % Indice ITAE: penalizza errori persistenti nel tempo
        J = trapz(t, t .* sqrt(sum(e.^2, 2)));

    catch ME
        fprintf('Si è verificato un errore: %s\n', ME.message);
        J = 1e10;
    end
end

%% Considerazioni risultati

