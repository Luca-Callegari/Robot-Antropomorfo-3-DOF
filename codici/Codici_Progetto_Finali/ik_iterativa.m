function [t_ik, q_ik] = ik_iterativa(qinit, XYZtarget, f_func, jac_func, ...
                                       nDOF, nCoords, tf_pd, soglia, ...
                                       alpha, tol, max_iter)
% Valori default
if nargin < 9,  alpha    = 0.1;  end
if nargin < 10, tol      = 1e-3; end
if nargin < 11, max_iter = 1000; end

    lambda = 1e-4;   % damping per pseudoinversa smorzata (Levenberg-Marquardt)

    q_k       = qinit;
    q_history = zeros(max_iter, nDOF);

for iter = 1:max_iter
        q_cell = num2cell(q_k);
        fq     = reshape(f_func(q_cell{:}), nCoords, 1);
        err_k  = XYZtarget - fq;
        J_k    = reshape(jac_func(q_cell{:}), nCoords, nDOF);

% ------------------------------------------------------------------
%  Pseudoinversa smorzata (Damped Least Squares / Levenberg-Marquardt)
%  delta_q = J' * (J*J' + lambda^2 * I)^{-1} * err
%  Vantaggi rispetto a J'*err:
%    - converge indipendentemente dalla scala della Jacobiana
%    - gestisce automaticamente le singolarita'
%    - lambda piccolo => vicino a pseudoinversa pura
%    - lambda grande  => vicino al gradiente puro (piu' stabile)
% ------------------------------------------------------------------
        A       = J_k * J_k' + lambda^2 * eye(nCoords);
        delta_q = J_k' * (A \ err_k);

        q_k = q_k + alpha * delta_q;
        q_history(iter, :) = q_k';

        if norm(err_k) < tol
            fprintf('IK iterativa convergita in %d iterazioni  |  ||err|| = %.2e\n', ...
                    iter, norm(err_k));
            q_history = q_history(1:iter, :);
            break;
        end

        if iter == max_iter
            fprintf('IK iterativa: max_iter=%d raggiunto  |  ||err|| = %.4f\n', ...
                    max_iter, norm(err_k));
        end
end

% Mappa le iterazioni sull'asse temporale [0, tf_pd]
    n_iter_done = size(q_history, 1);
    t_ik        = linspace(0, tf_pd, n_iter_done)';
    q_ik        = q_history;
end

%% Funzionava per i valori non reali del robot, nel caso di valori reali no a causa delle dimensioni piccole

% function [t_ik, q_ik] = ik_iterativa(qinit, XYZtarget, f_func, jac_func, ...
%                                        nDOF, nCoords, tf_pd,soglia, ...
%                                       alpha, tol, max_iter)
% 
%     % Valori default
%     if nargin < 9,  alpha    = 0.1;   end
%     if nargin < 10, tol      = 1e-3;  end
%     if nargin < 11, max_iter = 1000; end
% 
%     q_k       = qinit;
%     q_history = zeros(max_iter, nDOF);
% 
%     for iter = 1:max_iter
%         q_cell = num2cell(q_k);
%         fq     = reshape(f_func(q_cell{:}), nCoords, 1);
%         err_k  = XYZtarget - fq;
%         J_k    = reshape(jac_func(q_cell{:}), nCoords, nDOF);
% 
%         % ------------------------------------------------------------------
%         %  Gestione singolarita': ad ogni iterazione rivaluta il metodo
%         %  in base al determinante della Jacobiana corrente.
%         %  Se siamo vicini a una singolarita' (det piccolo) switcha
%         %  automaticamente al Gradiente anche se il metodo iniziale era Newton.
%         % ------------------------------------------------------------------
%         % if nDOF ~= nCoords
%         %     % Jacobiana rettangolare: sempre pseudoinversa
%         %     method_iter = 2;
%         % else
%         %     detJ = det(J_k);
%         %     if abs(detJ) > soglia
%         %         method_iter = 1;   % Newton: zona regolare
%         %     else
%         %         method_iter = 0;   % Gradiente: vicino a singolarita'
%         %     end
%         % end
% 
%         % switch method_iter
%         %     case 1,  delta_q = J_k \ err_k;
%         %     case 0,  delta_q = J_k' * err_k;
%         %     case 2,  delta_q = pinv(J_k) * err_k;
%         % end
% 
%         % Formula standard imparata grazie a Cristofari
%         delta_q = J_k' * err_k;
%         q_k = q_k + alpha * delta_q;
%         q_history(iter, :) = q_k';
% 
%         if norm(err_k) < tol
%             fprintf('IK iterativa convergita in %d iterazioni  |  ||err|| = %.2e\n', ...
%                     iter, norm(err_k));
%             q_history = q_history(1:iter, :);
%             break;
%         end
% 
%         if iter == max_iter
%             fprintf('IK iterativa: max_iter=%d raggiunto  |  ||err|| = %.4f\n', ...
%                     max_iter, norm(err_k));
%         end
%     end
% 
%     % Mappa le iterazioni sull'asse temporale [0, tf_pd]
%     n_iter_done = size(q_history, 1);
%     t_ik        = linspace(0, tf_pd, n_iter_done)';
%     q_ik        = q_history;
% end