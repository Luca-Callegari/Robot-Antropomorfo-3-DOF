%% Versione 2 corretta con i sistemi di riferimento adottati in Robotica industriale

function [T, U] = computeEnergies(DH, params)
% =========================================================================
%  COMPUTEENERGIES  -  Energia cinetica T e potenziale U
% =========================================================================
%  CONVENZIONE DH:  [ theta_i | d_i | alpha_i | a_i ]
%  Tutti i giunti sono ROTOIDALI. Robot 3D: gravita' lungo Z globale.
%
%  METODO JACOBIANO (da libro):
%  ----------------------------
%  Per ogni link i:
%    T_i = (1/2) * q_dot' * B_i * q_dot
%    B_i = Bt_i + Br_i
%
%  Energia cinetica di traslazione:
%    Bt_i = m_i * Jt_i' * Jt_i
%    Jt_i(:,j) = d(p_cm_i)/d(q_j)   per j<=i,  0 altrimenti
%
%  Energia cinetica di rotazione:
%    Br_i = Jw_i' * Ri * Ii * Ri' * Jw_i
%    Jw_i(:,j) = R_{0}^{j-1} * e_z  per j<=i,  0 altrimenti
%    Ri = matrice di rotazione del frame i rispetto al frame 0
%
%  Tensore di inerzia Ii (asta uniforme):
%    Link 1 (asta lungo Z): Ixx=Iyy=(1/12)*m1*d1^2,  Izz=0
%    Link 2,3 (asta lungo X): Ixx=0, Iyy=Izz=(1/12)*mi*li^2
%
%  Baricentro (punto medio del link):
%    Link 1: segmento lungo Z del frame base => p_cm = (0, 0, d1/2) globale
%    Link 2,3: segmento lungo X del frame i  => (a_i/2, 0, 0) nel frame i
%
%  Energia potenziale (gravita' lungo Z):
%    U_i = m_i * g * z_cm_i
%
%  INPUT:
%    DH     : matrice Nx4  [ theta_nom | d_i | alpha_i | a_i ]
%    params : struct  .nDOF, .m1..mN, .l1..lN, .g
%
%  OUTPUT:
%    T : energia cinetica totale (simbolica)
%    U : energia potenziale totale (simbolica)
% =========================================================================
    nDOF = params.nDOF;
    q_sym  = sym('q',  [nDOF 1], 'real');
    dq_sym = sym('dq', [nDOF 1], 'real');
    masses  = zeros(1, nDOF);
    lengths = zeros(1, nDOF);
    for i = 1:nDOF
        masses(i)  = params.(sprintf('m%d', i));
        lengths(i) = params.(sprintf('l%d', i));
    end
    g_val = params.g;
    e_z = [0; 0; 1];   % asse di rotazione di ogni giunto nel frame locale

    % Pre-calcolo trasformazioni cumulative T_0^i
    T_frames = cell(nDOF, 1);
    T_cumul  = sym(eye(4));
    for i = 1:nDOF
        d_i     = DH(i, 2);
        alpha_i = DH(i, 3);
        a_i     = DH(i, 4);
        Ti      = dh_matrix_en(q_sym(i), d_i, alpha_i, a_i);
        T_cumul = T_cumul * Ti;
        T_frames{i} = T_cumul;
    end

    T = sym(0);
    U = sym(0);

    for i = 1:nDOF
        m_i = masses(i);
        l_i = lengths(i);
        a_i = DH(i, 4);
        d_i = DH(i, 2);

        % Matrice di rotazione R_i (frame i rispetto al frame 0)
        Ri = T_frames{i}(1:3, 1:3);

        % ------- BARICENTRO -------
        % Link 1: segmento lungo Z del frame base, lungo d1
        %         il baricentro sta a meta' tra (0,0,0) e (0,0,d1)
        %         ed e' costante (sull'asse di rotazione q1 => Jt1=0)
        % Link 2,3: segmento lungo X del frame i, lungo a_i
        %           baricentro a (a_i/2, 0, 0) nel frame i
        if i == 1
            p_cm = [0; 0; d_i/2];
        else
            p_cm_h = T_frames{i} * [a_i/2; 0; 0; 1];
            p_cm   = p_cm_h(1:3);
        end

        % Jacobiana di posizione Jt_i (3 x nDOF)
        Jt = sym(zeros(3, nDOF));
        for j = 1:i
            Jt(:, j) = diff(p_cm, q_sym(j));
        end

        % Jacobiana di rotazione Jw_i (3 x nDOF)
        % Jw(:,j) = R_{0}^{j-1} * e_z  per j<=i
        Jw = sym(zeros(3, nDOF));
        for j = 1:i
            if j == 1
                Jw(:, j) = e_z;
            else
                Jw(:, j) = T_frames{j-1}(1:3, 1:3) * e_z;
            end
        end

        % ------- TENSORE DI INERZIA nel frame del link -------
        % Link 1: asta lungo Z => Ixx=Iyy=(1/12)*m1*d1^2, Izz=0
        % Link 2,3: asta lungo X => Ixx=0, Iyy=Izz=(1/12)*mi*li^2
        if i == 1
            I_body = diag(sym([(1/12)*m_i*d_i^2, ...
                                (1/12)*m_i*d_i^2, ...
                                0]));
        else
            I_body = diag(sym([0, ...
                                (1/12)*m_i*l_i^2, ...
                                (1/12)*m_i*l_i^2]));
        end

        % Matrici B di inerzia generalizzata
        Bt_i = m_i * (Jt.' * Jt);
        Br_i = Jw.' * Ri * I_body * Ri.' * Jw;
        B_i  = Bt_i + Br_i;

        % Energia cinetica link i: T_i = (1/2) * q_dot' * B_i * q_dot
        T = T + (sym(1)/2) * (dq_sym.' * B_i * dq_sym);

        % Energia potenziale link i: U_i = m_i * g * z_cm
        U = U + m_i * g_val * p_cm(3);   % z = componente 3 (robot 3D)
    end

    fprintf('Semplificazione di T e U in corso...\n');
    T = simplify(expand(T));
    U = simplify(U);
end

% -------------------------------------------------------------------------
function T = dh_matrix_en(theta, d, alpha, a)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    T  = [ ct, -st*ca,  st*sa, a*ct;
           st,  ct*ca, -ct*sa, a*st;
            0,     sa,     ca,    d;
            0,      0,      0,    1 ];
end


%% Versione precedente

