function draw_gas_plots(gas_measure, init, n_plot, x_axis, y_axis)

start(1) = gas_measure(init, 1)
start(2) = gas_measure(init, 2)



% Get the distance
last_dist=0
n_meas = 0
for i = init:length(gas_measure)
	
	x = gas_measure(i,1);
	y = gas_measure(i,2);
	dist = sqrt( (x- start(1))^2 + (y-start(2))^2 )
	if (last_dist==0) || ((dist - last_dist) > 0.1)
		n_meas = n_meas + 1;
		meas (n_meas, 1) = dist;
		meas (n_meas, 2) = gas_measure(i, n_plot);
		last_dist = dist
	end
end

plot (meas(:,1), meas(:,2), 'LineWidth', 3.0, 'Marker','o')
setLabelStyle(x_axis, y_axis)

end
