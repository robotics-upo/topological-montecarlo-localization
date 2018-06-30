function cellboxplot(cell_data)

data=[];
grp=[];

for i = 1:length(cell_data)
    data=[data, cell_data{i}];
    grp=[grp, ones(1, length(cell_data{i}))*i];
end

boxplot(data,grp)