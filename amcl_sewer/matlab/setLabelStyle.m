% Sets the style of one figure
function setLabelStyle(xlab, ylab, zlab)
  xlabel(xlab, 'fontsize',22);
  if (nargin > 1) 
    ylabel(ylab, 'fontsize', 22);
  end
  if (nargin > 2) 
      zlabel(zlab, 'fontsize', 22);
  end
      
  set(gca, 'FontSize', 22);
end