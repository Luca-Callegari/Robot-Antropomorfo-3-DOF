%% =========================================================================
%  ANALISI_ROBOT.m  –  Analisi completa del robot 3-DOF
%  =========================================================================
%  Qui vengono analizzate le vibrazioni del robot, la pulsazione naturale e
%  non naturale del sistema, fattore di smorzamento critico e non (tramite
%  i poli del sistema sia closed-loop sia open loop)
%  =========================================================================

%% =========================================================================
%  RECUPERO VARIABILI DAL WORKSPACE (generate da setup_simulink)
%  =========================================================================
nDOF = evalin('base','nDOF');
tf   = evalin('base','tf');
kp   = evalin('base','kp');
kd   = evalin('base','kd');
ki   = evalin('base','ki');
q0   = evalin('base','q0_pd');
dq0  = evalin('base','dq0_pd');
q_star = evalin('base','q_star');

fprintf('=== ANALISI ROBOT 3-DOF ===\n');
fprintf('q_star = [%.4f, %.4f, %.4f] rad\n', q_star);
fprintf('kp=%.1f  kd=%.1f  ki=%.1f\n\n', kp, kd, ki);


%% =========================================================================
%  SEZIONE 0: LINEARIZZAZIONE SIMBOLICA — stato x = [q; dq] (OPEN LOOP SENZA PID)
%  =========================================================================

%% =========================================================================
%  DIMOSTRAZIONE MATEMATICA DELLA MATRICE JACOBIANA A (SISTEMA MECCANICO)
%  =========================================================================
%
% 1. DEFINIZIONE DELLO STATO E DELLA FUNZIONE DI DINAMICA
%    Sia x lo stato del sistema: x = [x1; x2] = [q; dq] (dimensione 2n x 1)
%    La dinamica è espressa come: dx = f(x) = [f1(x1,x2); f2(x1,x2)]
%
%    Dove:
%    f1 = dq = x2  (Relazione cinematica)
%    f2 = ddq = M(x1)^-1 * [u(x1,x2) - C(x1,x2)*x2 - G(x1)] (Relazione dinamica)
%
% 2. LA MATRICE JACOBIANA A
%    Per definizione, A è la derivata parziale di f rispetto a x valutata
%    nel punto di equilibrio x* = [q_star; 0]:
%
%    A = [ df1/dx1 , df1/dx2 ]  = [ A11 , A12 ]
%        [ df2/dx1 , df2/dx2 ]    [ A21 , A22 ]
%
% 3. DIMOSTRAZIONE BLOCCHI CINEMATICI (A11, A12)
%    Essendo f1 = x2:
%    A11 = d(x2)/dx1 = 0  (La velocità non dipende esplicitamente dalla pos.)
%    A12 = d(x2)/dx2 = I  (Derivata di x2 rispetto a se stessa)
%
% 4. DIMOSTRAZIONE BLOCCO A21 (RIGIDEZZA)
%    Dobbiamo calcolare d(f2)/dx1 valutata in x*.
%    f2 = M(x1)^-1 * h(x1,x2)  dove h = [u - C*x2 - G]
%    Applicando la regola del prodotto (Leibniz):
%    df2/dx1 = [d(M^-1)/dx1 * h] + [M^-1 * dh/dx1]
%
%    Valutando in x* (equilibrio):
%    - x2 = 0 (velocità nulla)
%    - h(q_star, 0) = [u_eq - 0 - G(q_star)] = 0  (Le forze si bilanciano)
%    Quindi il primo termine [d(M^-1)/dx1 * 0] svanisce.
%
%    Resta: A21 = M^-1 * dh/dx1 = M^-1 * d(u - C*x2 - G)/dx1
%    Poiché x2=0 annulla il termine di Coriolis C:
%    A21 = M^-1 * [ du/dq - dG/dq ]
%    Con controllo PD u = Kp*(q_star - q) - Kd*dq:  du/dq = -Kp
%    => A21 = -M^-1 * (Kp + dG/dq)
%
% 5. DIMOSTRAZIONE BLOCCO A22 (SMORZAMENTO)
%    Dobbiamo calcolare d(f2)/dx2 valutata in x*.
%    df2/dx2 = [d(M^-1)/dx2 * h] + [M^-1 * dh/dx2]
%
%    - d(M^-1)/dx2 = 0 (L'inerzia M non dipende dalla velocità x2)
%    - dh/dx2 = d(u - C(x1,x2)*x2 - G(x1))/dx2
%
%    Derivando il termine di Coriolis C(x1,x2)*x2 rispetto a x2:
%    d(C*x2)/dx2 = C + (dC/dx2)*x2. Valutato in x2=0, questo termine è 0.
%
%    Resta solo la derivata del controllo u rispetto a x2:
%    du/dx2 = d(-Kp*errore - Kd*x2)/dx2 = -Kd
%    => A22 = -M^-1 * Kd
%
% 6. STRUTTURA FINALE (FORMA CANONICA)
%    A = [  0  ,  I  ]
%        [ A21 , A22 ]
% =========================================================================

% Avendo rimosso il PID dalla matrice A, ora vedo come reagisce la 
% struttura meccanica soggetta solo alla gravità e alla sua inerzia.
% Poiché ho linearizzato il sistema nell'equilibrio (senza PID), sto
% guardando come il robot si comporterebbe "da solo" se venisse spostato 
% leggermente dal target e lasciato libero (risposta libera).

fprintf('--- [1] LINEARIZZAZIONE SIMBOLICA IN x* = [q_eq; 0] ---\n');

% --- 1a. Definizione variabili simboliche (devono corrispondere a quelle
%         usate in setup_simulink per generare robot_*_generated) ---
syms q1 q2 q3 dq1 dq2 dq3 real
q_sym_loc  = [q1; q2; q3];
dq_sym_loc = [dq1; dq2; dq3];
state_sym  = [q_sym_loc; dq_sym_loc];   % vettore colonna [6x1]
q_eq = q_star; % Linearizzo rispetto alla q a regime (la q_star)

% --- 1b. M(q_eq) numerica: robot_M_generated accetta UN vettore [q;dq] ---
%         Verifica la firma: se accetta vettore → chiamata diretta,
%         se accetta scalari separati → usa num2cell
state_eq_num = [q_eq; zeros(nDOF,1)];
try
    M_eq = robot_M_generated(state_eq_num);
catch
    % La funzione potrebbe volere scalari separati
    cv = num2cell(state_eq_num);
    M_eq = robot_M_generated(cv{:});
end
Minv_eq = inv(M_eq);

fprintf('  M(q_eq) =\n'); disp(M_eq);

% --- 1c. G(q) simbolica e sua Jacobiana rispetto a q ---
%         robot_G_generated accetta [q;dq] come simbolici → risultato sym
try
    G_sym = robot_G_generated(state_sym);
catch
    cv_sym = num2cell(state_sym);
    G_sym  = robot_G_generated(cv_sym{:});
end

% Jacobiana simbolica dG/dq (matrice nDOF x nDOF)
dGdq_sym = jacobian(G_sym, q_sym_loc);

% Valutazione numerica in q_eq
subs_list = [q1, q2, q3, dq1, dq2, dq3];
subs_vals = [q_eq(1), q_eq(2), q_eq(3), 0, 0, 0];
dGdq_eq   = double(subs(dGdq_sym, subs_list, subs_vals));

fprintf('  dG/dq valutata in q_eq =\n'); disp(dGdq_eq);

%% SEZIONE 1: ANALISI DI STABILITÀ CON SISTEMA LINEARIZZATO (OPEN LOOP SENZA PID)
% E ANALISI DELLE FREQUENZE E SMORZAMENTI CRITICI

% --- 1d. Costruzione matrice A — stato x = [q; dq], dimensione 2n x 2n ---
O_n    = zeros(nDOF);
I_n    = eye(nDOF);
% Ho seguito le formule ricavate sopra a mano
A11 = O_n;
A12 = I_n;
A21 = -Minv_eq *  dGdq_eq;
A22 = O_n;

A_lin = [A11, A12; A21, A22];

fprintf('  Matrice A linearizzata (stato [q; dq], dimensione %dx%d):\n', 2*nDOF, 2*nDOF);
disp(A_lin);

% Autovalori -> poli del sistema linearizzato
poli = eig(A_lin);

fprintf('Poli del sistema linearizzato (dinamica senza PID):\n');
for i = 1:length(poli)
    %if imag(poli(i)) >= 0
        fprintf('  p%d = %.4f + %.4fi\n', i, real(poli(i)), imag(poli(i)));
    %end
end

% Stabilità: tutti i poli devono avere parte reale < 0
if all(real(poli) < 0)
    fprintf('  -> Sistema ASINTOTICAMENTE STABILE nel punto q*\n\n');
else
    fprintf('  -> ATTENZIONE: poli con parte reale >= 0 → instabilità locale!\n\n');
end

fprintf('--- FREQUENZE NATURALI E SMORZAMENTO ---\n');

% Estrai coppie coniugate: omega_n = |polo|, zeta = -Re/|polo|
poli_complessi = poli(imag(poli) > 1e-6);  
poli_reali     = poli(abs(imag(poli)) < 1e-6);

fprintf('Poli complessi coniugati (modi oscillatori):\n');
omega_n_list = [];
zeta_list    = [];
for i = 1:length(poli_complessi)
    p      = poli_complessi(i);
    % p = -zita*omega_n+-j*omega_n*sqrt(1-zita^2), so che omega_n =
    % sqrt(k/m) e se uno facesse i conti equivale proprio a fare la distanza
    % dall'origine, quindi il modulo del numero complesso
    omega_n = abs(p);
    % E' esattamente la formula standard, notando che
    % -real(p)=-(-zita*omega_n)
    zeta_i  = -real(p) / omega_n;
    % Formula standard di meccanica delle vibrazioni (parte di Valentini)
    omega_d  = omega_n * sqrt(max(0, 1 - zeta_i^2));
    % fn e fd sono proprio le due frequenze fatte col prof Valentini. La fn
    % è la frequenza a cui il robot "vorrebbe" oscillare naturalmente se 
    % non ci fosse alcuno smorzamento, mentre la fd è la frequenza che si
    % vede davvero durante la simulazione
    fn       = omega_n / (2*pi);
    fd       = omega_d / (2*pi);
    fprintf('  Modo %d: omega_n=%.4f rad/s (fn=%.4f Hz), zeta=%.4f, fd=%.4f Hz\n', ...
            i, omega_n, fn, zeta_i, fd);
    omega_n_list(end+1) = omega_n;
    zeta_list(end+1)    = zeta_i;
end

% Se $f_d$ coincide con le frequenze di risonanza della struttura: Il robot
% potrebbe iniziare a vibrare violentemente fino a rompersi.

if ~isempty(poli_reali)
    fprintf('Poli reali (modi aperiodici):\n');
    for i = 1:length(poli_reali)
        fprintf('  polo reale %d: %.4f  (tau=%.4f s)\n', i, poli_reali(i), -1/poli_reali(i));
    end
end


%% =========================================================================
%% SEZIONE 2: ANALISI DI STABILITÀ CON SISTEMA LINEARIZZATO (CLOSED LOOP PD)
% E ANALISI DELLE FREQUENZE E SMORZAMENTI CRITICI

% Posso riusare quello che ho dimostrato sopra siccome la u è data da un PD
% e non c'è il fattore integrativo che mi aumenta la dimensione dello stato

A11 = O_n;
A12 = I_n;
KP = kp*eye(nDOF);
KD = kd*eye(nDOF);
A21 = -Minv_eq * (KP + dGdq_eq);
A22 = -Minv_eq * KD;

A_lin = [A11, A12; A21, A22];

fprintf('  Matrice A linearizzata (stato [q; dq], dimensione %dx%d):\n', 2*nDOF, 2*nDOF);
disp(A_lin);

% Autovalori → poli del sistema linearizzato
poli = eig(A_lin);

fprintf('Poli del sistema linearizzato (dinamica con PD):\n');
for i = 1:length(poli)
    %if imag(poli(i)) >= 0
        fprintf('  p%d = %.4f + %.4fi\n', i, real(poli(i)), imag(poli(i)));
    %end
end

% Stabilità: tutti i poli devono avere parte reale < 0
if all(real(poli) < 0)
    fprintf('  -> Sistema ASINTOTICAMENTE STABILE nel punto q*\n\n');
else
    fprintf('  -> ATTENZIONE: poli con parte reale >= 0 → instabilità locale!\n\n');
end

fprintf('--- [2] FREQUENZE NATURALI E SMORZAMENTO ---\n');

% Estrai coppie coniugate: omega_n = |polo|, zeta = -Re/|polo|
poli_complessi = poli(imag(poli) > 1e-6);  % solo parte immaginaria > 0
poli_reali     = poli(abs(imag(poli)) < 1e-6);

fprintf('Poli complessi coniugati (modi oscillatori):\n');
omega_n_list = [];
zeta_list    = [];
for i = 1:length(poli_complessi)
    p      = poli_complessi(i);
    % p = -zita*omega_n+-j*omega_n*sqrt(1-zita^2), so che omega_n =
    % sqrt(k/m) e se uno facesse i conti equivale proprioa fare la distanza
    % dall'origine, quindi il modulo del numero complesso
    omega_n = abs(p);
    % E' esattamente la formula standard, notando che
    % -real(p)=-(-zita*omega_n)
    zeta_i  = -real(p) / omega_n;
    % Formula standard di meccanica delle vibrazioni (parte di Valentini)
    omega_d  = omega_n * sqrt(max(0, 1 - zeta_i^2));
    % fn e fd sono proprio le due frequenze fatte col prof Valentini. La fn
    % è la frequenza a cui il robot "vorrebbe" oscillare naturalmente se 
    % non ci fosse alcuno smorzamento, mentre la fd è la frequenza che si
    % vede davvero durante la simulazione
    fn       = omega_n / (2*pi);
    fd       = omega_d / (2*pi);
    fprintf('  Modo %d: omega_n=%.4f rad/s (fn=%.4f Hz), zeta=%.4f, fd=%.4f Hz\n', ...
            i, omega_n, fn, zeta_i, fd);
    omega_n_list(end+1) = omega_n;
    zeta_list(end+1)    = zeta_i;
end

% Se $f_d$ coincide con le frequenze di risonanza della struttura: Il robot
% potrebbe iniziare a vibrare violentemente fino a rompersi.

if ~isempty(poli_reali)
    fprintf('Poli reali (modi aperiodici):\n');
    for i = 1:length(poli_reali)
        fprintf('  polo reale %d: %.4f  (tau=%.4f s)\n', i, poli_reali(i), -1/poli_reali(i));
    end
end

% Smorzamento critico: zeta=1
M_diag = diag(M_eq);


fprintf('\nSmorzamento critico per ciascun giunto (sistema disaccoppiato):\n');
for j = 1:nDOF
    % Per non avere oscillazioni (zita = 1) serve che lo smorzatore
    % introdotto dal termine derivativo del pid si calcoli con questa
    % formula
    kd_crit = 2 * sqrt(kp * M_diag(j));
    % Ora calcolo la zita impostata da me, usando la definizione: zita=c/cr
    % che nel mio caso è kd/kd_crit
    zeta_approx = kd / kd_crit;

    if zeta_approx < 1
        stato = 'SOTTO-SMORZATO';
    elseif zeta_approx == 1
        stato = 'CRITICAMENTE SMORZATO';
    else
        stato = 'SOVRA-SMORZATO';
    end
    
    fprintf('  Giunto %d: kd_crit=%.2f,  kd_attuale=%.2f,  zeta_approx=%.3f  [%s]\n', ...
            j, kd_crit, kd, zeta_approx, stato);
end

% Se vedo un risultato tipo questo:
% Giunto 1: kd_crit=16.86,  kd_attuale=5.00,  zeta_approx=0.297  [SOTTO-SMORZATO]
% Giunto 2: kd_crit=17.70,  kd_attuale=5.00,  zeta_approx=0.282  [SOTTO-SMORZATO]
% Giunto 3: kd_crit=5.40,  kd_attuale=5.00,  zeta_approx=0.926  [SOTTO-SMORZATO]
% significa che sarebbe meglio definire un vettore kd=[kd1,kd2,kd3] così
% da associare ad ogni link uno smorzamento virtuale specifico, anziché
% associare ad ogni link lo stesso smorzamento virtuale. Infatti vediamo
% che ad esempio zeta_approx=0.282 nel giunto 2, è un zeta molto basso,
% quindi il link tenderà ad oscillare molto di più. La cosa ideale quindi
% sarebbe da definire kd2 = 0.7*kd_crit (del giunto 2) così da avere un
% buono smorzamento.
% Il fatto che ogni giunto "senta" un'inerzia diversa e sia influenzato 
% diversamente dalla gravità rende l'uso di un guadagno scalare identico 
% per tutti (kd uguale per tutti) una scelta inefficiente.
% Sempre con lo stesso esempio abbiamo ottenuto 
% Modo 1: omega_n=2.9658 rad/s (fn=0.4720 Hz), zeta=0.2966, fd=0.4508 Hz
% Modo 2: omega_n=3.0314 rad/s (fn=0.4825 Hz), zeta=0.2653, fd=0.4652 Hz
% Modo 3: omega_n=10.3231 rad/s (fn=1.6430 Hz), zeta=0.9261, fd=0.6200 Hz
% Questo ci fa vedere meglio come, data la zeta infima, le oscillazioni del
% primo e secondo link siano "elevate". Anche se comunque la frequenza che
% vediamo con i nostri occhi (fd) è relativamente bassa anche in quei due
% giunti. Questo si mostrerà Graficamente: Nello Scope di Simulink vedremo 
% delle onde sinusoidali molto larghe che impiegano diversi secondi a 
% estinguersi.

%% --- ESPERIMENTO DI RISONANZA SUL ROBOT LINEARIZZATO CON PD ---

% Definizione del Sistema nello Spazio di Stato (usando A)
C_sys = [eye(nDOF), zeros(nDOF)]; % Vogliamo vedere le uscite in posizione (q)
D_sys = zeros(nDOF, nDOF);
B_sys = [zeros(nDOF); Minv_eq];   % L'ingresso è la coppia ai giunti

% Creiamo il modello dinamico
sys_mimo = ss(A_lin, B_sys, C_sys, D_sys);

% Analisi qualitativa al variare di zeta
% Definiamo tre scenari di smorzamento:
% - Sotto-smorzato (Zeta << 1) -> Picco di risonanza violento
% - Smorzamento critico (Zeta = 1.0)   -> Nessuna risonanza (risposta piatta)
% - Sovra-smorzato (Zeta > 1.0)        -> Sistema molto lento (filtro passa-basso)

fprintf('\n--- ANALISI DELLA RISONANZA ---\n');
fprintf('Frequenza di eccitazione OMEGA: se OMEGA ≈ omega_n, il rapporto n=1.\n');
fprintf('All''aumentare di zeta, l''amplificazione dinamica M_d diminuisce:\n');
fprintf('M_d = 1 / sqrt( (1-n^2)^2 + (2*zeta*n)^2 )\n');

% 3. Plot della Risposta in Frequenza (Bode)
figure('Name', 'Analisi di Risonanza del Robot');

opzioni = bodeoptions;
opzioni.PhaseVisible = 'off'; % Nasconde la fase

figure('Name', 'Analisi dei Moduli - Risonanza MIMO');
bodeplot(sys_mimo, opzioni);
grid on;

title('Diagramma di Bode: Risposta dei Giunti a coppie sinusoidali');

% NOTA: I picchi che vedi nel grafico corrispondono alle tue omega_n.
% - Se i picchi sono alti e stretti: Zeta è basso (Risonanza).
% - Se i picchi spariscono: Zeta è alto (No Risonanza).
% Quello che vediamo è collegato strettamente con il fattore di
% amplificazione

%% =========================================================================
%  SEZIONE 3: ANALISI DI STABILITÀ CON SISTEMA LINEARIZZATO (CLOSED LOOP PID)
%  =========================================================================
%
%  Con PID completo si aumenta lo stato: x = [e; de; integral_e]
%  Dimensione 3n × 3n.
%
%% =========================================================================
%  DIMOSTRAZIONE ANALITICA DELLA MATRICE JACOBIANA A (CON CONTROLLO PID)
%  =========================================================================
%
% 1. DEFINIZIONE DELLO STATO ESTESO (AUGMENTED STATE)
%    Per includere l'azione integrale, aggiungiamo la variabile xi = integrale(e) dt.
%    Definiamo il nuovo vettore di stato x (dimensione 3n x 1):
%    x = [ x1 ] = [ q          ]  (Posizione)
%        [ x2 ] = [ dq         ]  (Velocità)
%        [ x3 ] = [ integrale_e]  (Errore integrato)
%
% 2. EQUAZIONI DELLA DINAMICA NEL TEMPO
%    La derivata dello stato dx = f(x) è composta da:
%    f1 = dx1 = x2                     (Cinematica: la derivata di q è dq)
%    f2 = dx2 = ddq = M^-1 * [u - G - C*dq] (Dinamica di Newton-Euler)
%    f3 = dx3 = (q_target - x1)        (Definizione dell'integrale dell'errore)
%
%    Nota: Per la linearizzazione in x*, consideriamo q_target = q_star,
%    quindi f3 = q_star - x1. All'equilibrio f3 = 0.
%
% 3. L'AZIONE DI CONTROLLO PID
%    u = Kp*(q_star - q) + Kd*(0 - dq) + Ki*(integrale_e)
%    Derivando u rispetto alle componenti dello stato:
%    du/dx1 = -Kp
%    du/dx2 = -Kd
%    du/dx3 =  Ki
%
% 4. COSTRUZIONE DELLA JACOBIANA A (3n x 3n)
%    A = [ df1/dx1  df1/dx2  df1/dx3 ]   [  0    I    0  ] (Blocco 1)
%        [ df2/dx1  df2/dx2  df2/dx3 ] = [ A21  A22  A23 ] (Blocco 2)
%        [ df3/dx1  df3/dx2  df3/dx3 ]   [ -I    0    0  ] (Blocco 3)
%
% 5. DIMOSTRAZIONE DEI BLOCCHI:
%
%    BLOCCO 1 (f1 = x2):
%    - df1/dx1 = 0, df1/dx2 = I, df1/dx3 = 0.
%
%    BLOCCO 2 (f2 = ddq):
%    Come nella dimostrazione PD, all'equilibrio i termini derivati di M^-1 
%    e il termine di Coriolis svaniscono. Resta:
%    A21 = M^-1 * d(u - G)/dq  = -M^-1 * (Kp + dG/dq)
%    A22 = M^-1 * d(u)/ddq     = -M^-1 * Kd
%    A23 = M^-1 * d(u)/dx3     =  M^-1 * Ki (Azione integrale sull'accelerazione)
%
%    BLOCCO 3 (f3 = q_star - x1):
%    - df3/dx1 = -I (Derivata di -x1 rispetto a x1)
%    - df3/dx2 =  0
%    - df3/dx3 =  0
%
% 6. STRUTTURA FINALE DELLA MATRICE A_PID:
%    A_PID = [      0,               I,            0      ]
%            [ -M^-1*(Kp + dG/dq), -M^-1*Kd,    M^-1*Ki   ]
%            [     -I,               0,            0      ]
%
% Nota sulla tua implementazione: La riga 3 della tua A_PID usa [I, 0, 0].
% Questo dipende dalla definizione del segno dell'errore (q - q_target).
% Se l'integratore accumula (q_target - q), allora df3/dx1 = -I.
% =========================================================================

fprintf('--- [3] ANALISI POLI SISTEMA CON PID COMPLETO ---\n');

% Usa M_eq e Minv_eq già calcolati nella Sezione 1 (simbolici, valutati in q_eq)
Ki_mat = ki * eye(nDOF);

O  = zeros(nDOF);
I  = eye(nDOF);

% trascurare i termini centrifughi $C(q, \dot{q})$ che sono quadratici 
% nella velocità.
A_PID = [  O_n,                           I_n,          O_n    ;
          -Minv_eq*(KP + dGdq_eq),  -Minv_eq*KD, -Minv_eq*Ki_mat;
           I_n,                           O_n,            O_n    ];

poli_PID = eig(A_PID);
fprintf('Poli sistema PID aumentato (3n=%d poli):\n', 3*nDOF);
for i = 1:length(poli_PID)
    p = poli_PID(i);
    fprintf('  p%d = %+.4f %+.4fi  |p|=%.4f', i, real(p), imag(p), abs(p));
    if imag(p) > 1e-6
        on  = abs(p);
        zt  = -real(p)/on;
        fprintf('  -> fn=%.3f Hz, zeta=%.3f', on/(2*pi), zt);
    end
    fprintf('\n');
end

if all(real(poli_PID) < 0)
    fprintf('  -> Sistema PID STABILE localmente in q*\n\n');
else
    n_instabili = sum(real(poli_PID) >= 0);
    fprintf('  -> ATTENZIONE: %d poli instabili!\n\n', n_instabili);
end

%% --- ESPERIMENTO DI RISONANZA SUL ROBOT LINEARIZZATO CON PID ---

% Nuova dimensione dello stato: 3 * nDOF
O_n = zeros(nDOF);
I_n = eye(nDOF);

% Definizione Matrice B (Ingresso: Coppia ai giunti)
% La coppia influenza direttamente solo l'accelerazione (secondo blocco di righe)
B_PID = [ O_n ; 
          Minv_eq ; 
          O_n ];

% Definizione Matrice C (Uscita: Posizione dei giunti)
% Vogliamo leggere q, che sono le prime n variabili di stato
C_PID = [ I_n, O_n, O_n ]; 

% Creazione del sistema
D_PID = zeros(nDOF, nDOF);
sys_pid = ss(A_PID, B_PID, C_PID, D_PID);

opzioni = bodeoptions;
opzioni.PhaseVisible = 'off';
figure('Name', 'Bode con PID completo');
bodeplot(sys_pid, opzioni);
grid on;

% I valori intorno a -400 dB o -300 dB sono numericamente equivalenti a zero.
% Significato fisico: Significa che i giunti sono quasi totalmente 
% disaccoppiati a quelle frequenze. Ad esempio, nel grafico In(1) -> Out(3), 
% una coppia applicata alla base (Giunto 1) non ha quasi alcun effetto 
% misurabile sulla posizione dell'ultimo link (Giunto 3).