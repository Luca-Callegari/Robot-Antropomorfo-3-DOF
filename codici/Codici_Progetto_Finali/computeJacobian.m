function jac = computeJacobian(f, q_sym)
% =========================================================================
%  COMPUTEJACOBIAN  -  Jacobiana simbolica  (3 x N)
% =========================================================================
%  Calcola la Jacobiana di posizione:  jac(i,j) = d f_i / d q_j
%
%  Equivalente Mathematica:
%    jac = FullSimplify[D[f[q1,q2,q3], {variables}]]
%
%  INPUT:
%    f      : vettore simbolico 3x1  (da computeKinematics)
%    q_sym  : vettore simbolico Nx1  [q1; ...; qN]
%
%  OUTPUT:
%    jac    : matrice simbolica 3xN
% =========================================================================

    % Definisco la matrice Jacobiana come una matrice simbolica composta da
    % tutti zeri, facendo attenzione che il numero di righe è pari alla
    % dimensione del vettore di funzioni (f) e il numero di colonne è pari
    % al numero di variabili di giunto 
    M   = length(f);      
    N   = length(q_sym);
    jac = sym(zeros(M, N));

    % Per ogni riga e per ogni colonna mi calcolo la derivata parziale
    % corrispondente
    for i = 1:M
        for j = 1:N
            jac(i, j) = diff(f(i), q_sym(j));
        end
    end

    % Semplifico al massimo che riesco la matrice Jacobiana
    jac = simplify(jac);
end
