% Rearrange the matrix into a 4x4 matrix
rearrangedMatrix = reshape(rmse_values, 4, 4);

rmse_values = round(rearrangedMatrix, 3);

% Write the numerical data to Excel spreadsheet using writetable
writematrix(rmse_values);
