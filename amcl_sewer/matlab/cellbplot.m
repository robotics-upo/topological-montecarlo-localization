function cellboxplot(cell_data)


hold on;
for i = 1:length(cell_data)
  bplot(cell_data{i}, i)
end

hold off