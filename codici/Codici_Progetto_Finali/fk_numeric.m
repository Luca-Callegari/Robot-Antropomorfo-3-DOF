function ee = fk_numeric(q, DH)
% =========================================================================
%  FK_NUMERIC  -  Cinematica diretta numerica (robot 3D, giunti rotoidali)
% =========================================================================
%  CONVENZIONE DH:  [ theta_nom | d_i | alpha_i | a_i ]
%  q_i sostituisce theta_i. Output sempre [x; y; z].
%
%  INPUT:
%    q   : vettore Nx1 angoli di giunto [rad]
%    DH  : matrice Nx4
%
%  OUTPUT:
%    ee  : posizione end-effector [x; y; z]
% =========================================================================

    nDOF    = size(DH, 1);
    T_total = eye(4);

    for i = 1:nDOF
        d_i     = DH(i, 2);
        alpha_i = DH(i, 3);
        a_i     = DH(i, 4);
        theta_i = q(i);

        ct = cos(theta_i); st = sin(theta_i);
        ca = cos(alpha_i); sa = sin(alpha_i);

        Ti = [ ct, -st*ca,  st*sa, a_i*ct;
               st,  ct*ca, -ct*sa, a_i*st;
                0,     sa,     ca,    d_i;
                0,      0,      0,      1 ];

        T_total = T_total * Ti;
    end

    ee = T_total(1:3, 4);
end
