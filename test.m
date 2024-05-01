

% Define the time variable t
t = linspace(0, 10, 100); % You can adjust the time range and resolution as needed

% Compute the expression z(t)
z = t-s(exp(-t));



plot(t, z);
xlabel('Time (t)');
ylabel('z(t)');
title('z(t) = e^-t');
grid on;