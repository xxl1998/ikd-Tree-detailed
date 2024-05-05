%https://community.amd.com/sdtpp67534/attachments/sdtpp67534/devgurus-archive-discussions/37215/1/Matthew%20Scarpino%20%E2%80%94%20OpenCL%20in%20Action%20%E2%80%94%202011.pdf.pdf
%https://link.springer.com/content/pdf/10.1007/978-1-4842-4398-5.pdf
%https://www.intel.com/content/www/us/en/docs/onetbb/get-started-guide/2021-12/overview.html

clear;

function check_solution_error(A, x, b)
  residual = b - A * x;
  residual_norm = norm(residual);
  fprintf('The norm of the residual (error) is: %f\n', residual_norm);
end

function show_distance(A, x)
  % Define plane equation as ax + by + cz + d = 0
  d = 1;
  % Calculate distances from each point to the plane defined by ax + by + cz + d = 0
  % Distance formula is |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
  num_points = size(A, 1);
  distances = zeros(num_points, 1);
  for i = 1:num_points
    point = A(i, :);
    distance = abs(x(1)*point(1) + x(2)*point(2) + x(3)*point(3) + d) / norm(x);
    distances(i) = distance;
  end
  % Print distances
  disp('Distances of each point to the plane:');
  disp(distances);
  [max_distance, idx] = max(distances);
  fprintf('The maximum distance is: %f, at index %d\n', max_distance, idx);
end

b = [-1; -1; -1; -1; -1];

A = [-2.276, 0.4507, 0.0329;
     -2.273, 0.5457, 0.08334;
     -2.71, 0.2871, -0.05081;
     -2.871, 0.5307, 0.08712;
     -2.337, 0.2689, -0.1338];
x = [0.1688; -1.404; 1.957];
A_square = zeros(5, 5);
A_square(:, 1:3) = A;

A_1 = [  9.775, -0.2636, -0.1972;
        9.235, -0.2776, -0.1902;
        9.746,  0.2444,  -0.193;
         9.26,  0.2579, -0.1672;
        9.731, -0.7539, -0.2024];
x_1= [-0.1368; 0.01186; -1.623];


printf("\n\neigen solve,");
check_solution_error(A, x, b);
show_distance(A,x);

[Q, R] = qr(A);
x_qr = R \ (Q' * b);
printf("\n\nOctave QR decomposition solve,");
check_solution_error(A, x_qr, b);
show_distance(A,x_qr);

x_div = A\b;
printf("\n\nOctave division solve,");
check_solution_error(A, x_div, b);
show_distance(A,x_div);
