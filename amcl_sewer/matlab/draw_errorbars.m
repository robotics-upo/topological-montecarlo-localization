function getManholeStats(v, dev, disp)

figure
%  v = [0.185157 0.249656 0.497677 0.258	 0.182646 1.25118	 1.47323	0.249421 0.394154 0.706092 1.1865	]
manholes = [1:length(v)]
%  dev = [0.0500855
%  0.0764993
%  0.299349
%  0.0626562
%  0.0982856
%  1.05405
%  0.245071
%  0.0854754
%  0.370116
%  0.495471
%  1.27061]
bar (manholes, v)
hold on
errorbar(manholes, v, dev, 'xr')
setLabelStyle('Visited Manhole', 'Mean distance to manhole (m)')

figure

plot(manholes, disp, 'LineWidth', 3)
setLabelStyle('Visited Manhole', 'Standard Deviation of the set of particles (m)')
