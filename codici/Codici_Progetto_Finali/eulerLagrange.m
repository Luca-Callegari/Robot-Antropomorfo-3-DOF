function EL = eulerLagrange(T, U, q_sym)
% =========================================================================
%  EULERLAGRANGE  -  Equazioni del moto tramite Eulero-Lagrange
% =========================================================================
%  Ricava le N equazioni del moto:
%    d/dt( dL/d(dq_i) ) - dL/dq_i = tau_i    con  L = T - U
%
%  Equivalente alle eqss del notebook Mathematica originale.
%
%  INPUT:
%    T      : energia cinetica simbolica   (da computeEnergies)
%    U      : energia potenziale simbolica (da computeEnergies)
%    q_sym  : vettore simbolico [q1;...;qN] (da computeKinematics)
%
%  OUTPUT:
%    EL.lhs   : vettore Nx1  lato sinistro eq. i  (= tau_i)
%    EL.M     : matrice di inerzia M(q)       [NxN]
%    EL.C_vec : vettore Coriolis+centrifuga   [Nx1]
%    EL.G     : vettore gravitazionale        [Nx1]
%    EL.q     : q_sym
%    EL.dq    : dq_sym
%    EL.ddq   : ddq_sym
% =========================================================================

    % Definisco un vettore simbolico di variabili che mi compariranno nelle
    % E-L
    nDOF    = length(q_sym);
    dq_sym  = sym('dq',  [nDOF 1], 'real');
    ddq_sym = sym('ddq', [nDOF 1], 'real');

    % Definisco il Lagrangiano nel modo classico per i sistemi meccanici
    L = T - U;

    fprintf('Calcolo equazioni di Eulero-Lagrange (%d DOF)...\n', nDOF);

  
    lhs_vec = sym(zeros(nDOF, 1));
    M_mat   = sym(zeros(nDOF, nDOF));
    G_vec   = sym(zeros(nDOF, 1));

    for i = 1:nDOF
        % Calcolo per ogni variabile q_i il fattore (dL/ddq)
        dL_ddqi  = diff(L, dq_sym(i));

        ddt_term = sym(0);
        for j = 1:nDOF
            % Calcolo il fattore (d/dt)*(dL/ddq) usando regola della catena
            ddt_term = ddt_term ...
                     + diff(dL_ddqi, q_sym(j))  * dq_sym(j) ...
                     + diff(dL_ddqi, dq_sym(j)) * ddq_sym(j);
        end

        % Avendo trovato il valore simbolico (d/dt)*(dL/ddq) lo sottraggo a
        % dL/dq scrivendo in modo identico le E-L. Ricordo che nelle E-L
        % abbiamo uguaglianza a zero, quindi abbiamo trovato il membro
        % sinistro dell'uguaglianza i-esima
        lhs_i      = simplify(ddt_term - diff(L, q_sym(i)));
        lhs_vec(i) = lhs_i;

        % Estrazione matrice di inerzia (fissata la i mi svolgo tutta la
        % colonna). In pratica ad ogni passo "i" mi sto trovando riga per
        % riga la matrice di inerzia
        for j = 1:nDOF
            M_mat(i,j) = simplify(diff(lhs_i, ddq_sym(j)));
        end

        % Estrazione vettore gravitazionale G(q) che non dipende né da dq
        % né ddq. Siccome lhs_i = M(q)*ddq + C(q,dq)*dq + G(q) per ogni i,
        % allora il trucco per trovare la G(q) è quello di prendere la
        % lhs_i e di imporre dq=0 e ddq=0. Notare che sto imponendo 2*nDOF
        % zeri perché ho [ dq1, dq2, ..., dqN, ddq1, ddq2, ..., ddqN ]
        G_vec(i) = simplify(subs(lhs_i, [dq_sym; ddq_sym], zeros(2*nDOF, 1)));

        fprintf('  equazione %d/%d completata\n', i, nDOF);
    end

    % Definito per ogni equazione la matrice di inerzia M e il vettore
    % gravitazionale G_vec, posso trovare il vettore definito da C(q,dq)*dq
    % esplicitandolo dall'equazione lhs_i = M(q)*ddq + C(q,dq)*dq + G(q)
    C_vec = simplify(lhs_vec - M_mat * ddq_sym - G_vec);

    % Salvo tutto quanto in un oggetto del tipo EL
    EL.lhs   = lhs_vec;
    EL.M     = M_mat;
    EL.C_vec = C_vec;
    EL.G     = G_vec;
    EL.q     = q_sym;
    EL.dq    = dq_sym;
    EL.ddq   = ddq_sym;
end
