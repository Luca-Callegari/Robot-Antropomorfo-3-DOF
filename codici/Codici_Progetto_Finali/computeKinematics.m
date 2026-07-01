function [f, q_sym] = computeKinematics(DH)
% =========================================================================
%  COMPUTEKINEMATICS  -  Cinematica diretta simbolica
% =========================================================================
%  CONVENZIONE DH (colonne):  [ theta_i | d_i | alpha_i | a_i ]
%
%  Tutti i giunti sono ROTOIDALI: q_i sostituisce theta_i.
%  Il robot e' 3D: f restituisce sempre [x; y; z].
%
%  La matrice omogenea DH standard per ogni link e':
%    T_i = Rz(q_i) * Tz(d_i) * Rx(alpha_i) * Tx(a_i)
%
%  INPUT:
%    DH     : matrice Nx4  [ theta_nom | d_i | alpha_i | a_i ]
%             
%
%  OUTPUT:
%    f      : vettore simbolico 3x1  [x; y; z]  posizione end-effector
%    q_sym  : vettore simbolico Nx1  [q1; q2; ...; qN]
% =========================================================================

    % Il numero di righe della DH è proprio il numero di gradi di libertà
    nDOF  = size(DH, 1);
    
    % Creo una variabile simbolica q = [q1, q2], dove q1 e q2 sono due
    % simboli ma che necessariamente rappresenteranno numeri reali
    q_sym = sym('q', [nDOF 1], 'real');

    % Creo una matrice identità simbolica. Normalmente se facessi eye(4) mi
    % genererebbe una matrice identità 4x4. Ma se tale matrice deve
    % contenere elementi simbolici (tipo q1 e q2) mi serve che anche questa
    % matrice identità sia simbolica
    T_total = sym(eye(4));

    % Per ogni riga della matrice DH mi prendo il valore (d,alpha,a) e mi
    % calcolo la matrice di cambiamento delle coordinate dal link 1 a 2,
    % poi da 1 a 3, poi da 1 a 4 e così via
    for i = 1:nDOF
        d_i     = DH(i, 2);
        alpha_i = DH(i, 3);
        a_i     = DH(i, 4);
        Ti      = dh_matrix(q_sym(i), d_i, alpha_i, a_i);
        T_total = T_total * Ti;
    end

    % Con T_total(1:3, 4) estraggo la posizione dell'end-effector, quindi
    % mi estraggo la cinematica diretta del robot
    f = simplify(T_total(1:3, 4));
end

% -------------------------------------------------------------------------
% Generica matrice di cambiamento di coordinate dal giunto i a i+1
function T = dh_matrix(theta, d, alpha, a)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    T  = [ ct, -st*ca,  st*sa, a*ct;
           st,  ct*ca, -ct*sa, a*st;
            0,     sa,     ca,    d;
            0,      0,      0,    1 ];
end
