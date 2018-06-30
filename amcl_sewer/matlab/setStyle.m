% Sets the style of one figure
function setLabelStyle(xlab, ylab)
  xlabel(xlab, 'fontsize',24);
  ylabel(ylab, 'fontsize', 24);
  set(gca, 'FontSize', 24);
end