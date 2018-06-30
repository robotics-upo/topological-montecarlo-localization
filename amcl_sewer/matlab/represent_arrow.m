function represent_arrow(plan, dimen, color, shift_m)
  % Represent an arrow that starts in the first waypoint
  % And goes to the second. It will be shifted in the perpendicular
  % direction
      
  start = plan(1,1:dimen)
  seg = plan(2,1:dimen) - plan(1,1:dimen);
  length_ = seg * 0.2;
  if (dimen == 2) 
    shift = [ -seg(2) seg(1) ];
    shift = shift / norm(shift) * 0.5;
  else 
    shift = [ -seg(2) seg(1) 0];
    shift = shift / norm(shift) * 0.5
  end
  shift = shift * shift_m;
      
  start = start + shift
  edge = start + length_;
  arrow(start, edge, 'FaceColor', color, 'EdgeColor', color, 'LineWidth', 2)  
end  