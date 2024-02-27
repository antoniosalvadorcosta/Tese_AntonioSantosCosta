% Rearrange the matrix into a 4x4 matrix

disp(rmse_values_wind);
rearrangedMatrix = reshape(rmse_values_wind, 4, 12);

rmse_values_wind = round(rearrangedMatrix, 3);

% Write the numerical data to Excel spreadsheet using writetable
writematrix(rmse_values_wind);f & %f & %f\n',rmse_values(4,1),rmse_values(4,2),rmse_values(4,3),rmse_values(4,4));