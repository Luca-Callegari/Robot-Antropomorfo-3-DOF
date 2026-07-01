%% =========================================================================
%  FASE 2 - ADDESTRAMENTO RETE NEURALE  (Lo "Studente")
%  =========================================================================
%  Addestra una rete feedforward che impara la mappatura:
%    [q1, q2, q3] -> [kp_opt, kd_opt, ki_opt]
%
%  La rete ha architettura:  3 -> 12 -> 3
%    - Input:  configurazione di giunto q [3x1]
%    - Hidden: 12 neuroni con attivazione tanh
%    - Output: guadagni PID ottimali [kp, kd, ki]
%
%  PRIMA DI LANCIARE: commentare i blocchi constant contenenti kp_sim,
%  kd_sim, ki_sim utilizzati nella fase 1 di ottimizzazione
%
%  I pesi vengono salvati in 'parametri_rete.mat' con dimensioni esplicite
%  per compatibilita' con coder.load() nei blocchi MATLAB Function Simulink.
% =========================================================================

clear; clc;

%% Carica dataset generato dalla Fase 1
if ~exist('dataset_neurale.mat', 'file')
    error('dataset_neurale.mat non trovato. Esegui prima fase1_ottimizzazione.m');
end
load('dataset_neurale.mat');   % carica X_input [Nx3], Y_target [Nx3]

fprintf('Dataset caricato: %d campioni\n', size(X_input, 1));
fprintf('Input  (q):         min=[%.2f %.2f %.2f]  max=[%.2f %.2f %.2f]\n', ...
        min(X_input), max(X_input));
fprintf('Output (kp,kd,ki):  min=[%.2f %.2f %.2f]  max=[%.2f %.2f %.2f]\n', ...
        min(Y_target), max(Y_target));

%% Architettura rete
n_hidden1 = 16;   % neuroni nello strato nascosto 1 (unico strato nascosto)

% Creo la rete specificando numeri di layer nascosti e funzione di
% addestramento
net = feedforwardnet([n_hidden1], 'trainbr');

% Disabilita preprocessing automatico (normalizzazione):
% lo facciamo noi manualmente per avere controllo totale
% e per semplificare l'implementazione in Simulink
net.inputs{1}.processFcns  = {};
net.outputs{end}.processFcns = {};

% Divisione dati: 70% train, 15% validation, 15% test
net.divideParam.trainRatio = 0.70;
net.divideParam.valRatio   = 0.15;
net.divideParam.testRatio  = 0.15;

% Imposta il numero massimo di epoche
net.trainParam.epochs = 1000; 

net.trainParam.goal = 1e-6;    % Obiettivo di errore (Mean Squared Error)
net.trainParam.lr = 0.001;      % Learning rate (tasso di apprendimento)

% Normalizzazione manuale
X_mean = mean(X_input);
X_std  = std(X_input);
X_norm = (X_input - X_mean) ./ X_std;

Y_mean = mean(Y_target);
Y_std  = std(Y_target);
Y_std(Y_std < 1e-6) = 1;
Y_norm = (Y_target - Y_mean) ./ Y_std;

fprintf('Normalizzazione output:\n');
fprintf('  Y_mean: kp=%.4f  kd=%.4f  ki=%.4f\n', Y_mean);
fprintf('  Y_std:  kp=%.4f  kd=%.4f  ki=%.4f\n', Y_std);

% Training
fprintf('\nAvvio training...\n');
%[net, tr] = train(net, X_input', Y_target');
[net, tr] = train(net, X_norm', Y_norm');

%% Valutazione performance
% Y_pred = net(X_input')';   
% rmse   = sqrt(mean((Y_pred - Y_target).^2));
% fprintf('\nRMSE per guadagno:\n');
% fprintf('  kp: %.4f\n  kd: %.4f\n  ki: %.4f\n', rmse);

Y_pred_norm = net(X_norm')';
Y_pred = Y_pred_norm .* Y_std + Y_mean;
rmse = sqrt(mean((Y_pred - Y_target).^2));
fprintf('\nRMSE in scala originale:\n');
fprintf('  kp: %.4f\n  kd: %.4f\n  ki: %.4f\n', rmse);

%% Estrazione pesi con dimensioni esplicite
%  IMPORTANTE: le dimensioni devono essere fisse per coder.load() in Simulink
W1 = net.IW{1,1};   
b1 = net.b{1};       
W2 = net.LW{2,1};    
b2 = net.b{2};                    

fprintf('\nDimensioni pesi estratti:\n');
fprintf('  W1: %dx%d\n', size(W1));
fprintf('  b1: %dx%d\n', size(b1));
fprintf('  W2: %dx%d\n', size(W2));
fprintf('  b2: %dx%d\n', size(b2));

% Verifica che W2 non sia vuoto
if isempty(W2)
    error('W2 e'' vuoto — controlla la configurazione della rete.');
end


%% Salvataggio parametri
% save('parametri_rete.mat', 'W1', 'b1', 'W2', 'b2');
% fprintf('\nParametri rete salvati in parametri_rete.mat\n');

save('parametri_rete.mat', 'W1', 'b1', 'W2', 'b2', ...
     'X_mean', 'X_std', 'Y_mean', 'Y_std');
fprintf('\nParametri e normalizzazione salvati in parametri_rete.mat\n');

%% Test manuale: confronto rete vs dataset
% fprintf('\nConfronto rete vs dataset (primi 5 campioni):\n');
% fprintf('%-20s %-20s %-20s\n', 'q [rad]', 'kp,kd,ki reali', 'kp,kd,ki predetti');
% for i = 1:min(5, size(X_input,1))
%     y_r = Y_target(i,:);
%     y_p = net(X_input(i,:)')';
%     fprintf('[%.2f %.2f %.2f]  [%.1f %.1f %.1f]  [%.1f %.1f %.1f]\n', ...
%             X_input(i,:), y_r, y_p);
% end

fprintf('\nConfronto rete vs dataset (primi 5 campioni):\n');
for i = 1:min(5, size(X_input,1))
    y_r = Y_target(i,:);
    y_p = net(X_norm(i,:)')' .* Y_std + Y_mean;
    fprintf('[%.2f %.2f %.2f]  reali=[%.1f %.1f %.1f]  pred=[%.1f %.1f %.1f]\n', ...
            X_input(i,:), y_r, y_p);
end

%% Visualizzazione rete
%view(net);


