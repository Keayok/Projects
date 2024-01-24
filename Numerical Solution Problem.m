% Code used to Import Cp data from file

CpData = importdata('CpData.txt').data;
Cp = CpData(:, 2)';
Temperature = CpData(:, 1)';


%These are the known values that will be used to test T2_value 
Q = 300000; M = 1; T1 = 450;


% The user will be prompted for an initial guess for T2

while true
   prompt = 'Enter an initial value to guess T2:';
   T2_value = input(prompt);
    
    if T2_value >= CpData(1, 1) && T2_value <= CpData(end, 1)
        break; % Exit the loop if the input T2 value is within range
    else
        fprintf('Invalid input for T2. Enter a value within the Cp data range.\n');
    end
end

%The code to initialise both the iteration function and iteration matrix

Max_iterations = 500; % Assume that 500 is the maximum amount of iterations 
Desired_error = 0.0005; % 0.05% expressed as a decimal for the acceptable relative error between numerical and exact solution

Iteration_matrix = zeros(Max_iterations, 2); % Preallocate Iteration_matrix
Iteration = 1;
error = 1;
alpha = 50; % alpha is a constant value that can be adjusted to affect the convergence speed and overall accuracy of the numerical solution.

% while loop to calculate Cp values at t1 and t2,Calculate the average Cp as well as the Qvalue and Error for each iteration unitl it satifies the conditions   
while abs(error) > Desired_error && Iteration <= Max_iterations
    Iteration_matrix(Iteration, :) = [Iteration, T2_value];
    
    
    Cp_at_T1 = interp1(Temperature, Cp, T1);
    Cp_at_T2_value = interp1(Temperature, Cp, T2_value);
    Cp_avg = mean([Cp_at_T1, Cp_at_T2_value]);

    Q2_value = M * Cp_avg * (T2_value - T1);
    
    error = (Q - Q2_value) / Q;
    
    Iteration = Iteration + 1;
    T2_value = T2_value + error * alpha;
end

% Code to Display and plot the results of the Numerical solution vs exact solution
fprintf('The Convergence is at a T2 value of %0.3f K, Iterations: %d\n', T2_value, Iteration - 1);
figure;

plot(Iteration_matrix(1:Iteration-1, 1), Iteration_matrix(1:Iteration-1, 2), "--r", 'LineWidth', 1);
hold on;

Exact_solution = 735.244; % Replace this with the actual value of the exact solution
plot([1, Iteration - 1], [Exact_solution, Exact_solution], "-.k", 'LineWidth', 1);

xlabel('Iteration Number');
ylabel('Temperature (K)');
title('Numerical Solution, \alpha = 50');

legend({'Numerical Solution', 'Exact Solution'}, 'Location', 'best');
grid on;
