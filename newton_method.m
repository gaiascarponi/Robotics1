function [q_out, guesses, cartesian_errors] = newton_method(q_in, desired_point, p, initial_guess, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)
% [q_out, guesses, cartesian_errors] = newton(q_in, desired_point, p, 
%  initial_guess, max_iterations, max_cartesian_error, min_joint_increment, 
%  max_closeness_singularity) takes as inputs:
%   -q_in: The variables of the joints, e.g. [q1;q2; q3; q4]
%   -desired_point: the configuration we wish to reach
%   -p: The mapping from joints to points (p=f(q))
%   -initial_guess: Initial configuration of joints
%   -max_iterations: The maximum number of iterations we can perform
%   -max_cartesian_error: The level of precision we wish to reach
%   -min_joint_increment: (Optional) The minimum level of increment of accuracy 
%                          between successive iterations. Default = 1e-5.
%   -max_closeness_singularity: (Optional) How close to a singularity we can get.
%                                Default = 1e-4.
% and outputs:
%   -q_out: The best reached configuration
%   -guesses: The history of tested configurations
%   -cartesian_errors: The history of errors

    % Set default values if optional arguments are not provided
    if nargin < 7
        min_joint_increment = 1e-5;
    end
    if nargin < 8
        max_closeness_singularity = 1e-4;
    end

    % Initialize variables
    guesses = zeros(max_iterations + 1, length(q_in));
    cartesian_errors = zeros(1, max_iterations + 1);
    guess = initial_guess;
    
    % Print table header
    fprintf('| iteration k | q^{[k]}          | p^{[k]}       | ||e^{[k]}|| | det J(q^{[k]}) |\n');
    fprintf('|-------------|------------------|--------------|------------|----------------|\n');
    
    % Iterative Newton-Raphson method
    for i = 1:max_iterations
        guesses(i, :) = guess;
        
        % Compute Cartesian error
        current_p = eval(subs(p, q_in, guess));
        error = norm(desired_point - current_p);
        cartesian_errors(i) = error;
        
        % Compute Jacobian and its determinant
        J = jacobian(p, q_in);
        J_val = subs(J, q_in, guess);
        det_J = det(J_val);
        
        % Print current iteration information
        fprintf('| %-11d | (%-.4f, %-.4f, %-.4f) | (%-.4f, %-.4f, %-.4f) | %-10.5f | %-14.4f |\n', ...
                i-1, guess(1), guess(2), guess(3), current_p(1), current_p(2), current_p(3), error, det_J);
        
        if error < max_cartesian_error
            fprintf("\nConverged at iteration %d (error < tolerance).\n", i);
            break
        end
        
        % Check for singularity and decide inverse method
        if abs(det_J) < max_closeness_singularity
            fprintf("\nWarning: Near singularity at iteration %d (|det(J)| = %e < %e), using pseudoinverse.\n", ...
                    i, abs(det_J), max_closeness_singularity);
            J_inv = pinv(J_val);
        else
            % Use regular inverse if well-conditioned
            J_inv = inv(J_val);
        end
        
        % Compute the next guess
        delta_q = J_inv * (desired_point - current_p);
        new_guess = guess + delta_q;
        
        % Check for minimal increment
        if norm(delta_q) <= min_joint_increment
            fprintf("\nStopped at iteration %d (joint increment too small).\n", i);
            break
        end
        
        guess = eval(new_guess);
    end
    
    % Print final iteration if converged
    if i <= max_iterations && error >= max_cartesian_error
        current_p = eval(subs(p, q_in, guess));
        J_val = subs(J, q_in, guess);
        det_J = det(J_val);
        fprintf('| %-11d | (%-.4f, %-.4f, %-.4f) | (%-.4f, %-.4f, %-.4f) | %-10.5f | %-14.4f |\n', ...
                i, guess(1), guess(2), guess(3), current_p(1), current_p(2), current_p(3), error, det_J);
    end
    
    % Trim outputs
    guesses = guesses(1:i, :);
    cartesian_errors = cartesian_errors(1:i);
    q_out = guess;
    
    % Final status report
    final_error = cartesian_errors(end);
    if final_error < max_cartesian_error
        fprintf("\nSuccessfully converged with error %e < %e\n", final_error, max_cartesian_error);
    else
        fprintf("\nStopped after %d iterations with error %e (target: %e)\n", ...
                i, final_error, max_cartesian_error);
    end
    
    fprintf("Final joint configuration:\n");
    disp(q_out);
    fprintf("Final position:\n");
    disp(eval(subs(p, q_in, q_out)));
end