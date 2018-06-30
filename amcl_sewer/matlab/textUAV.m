getUAVText = function (n_uavs, text)
   for i=1:n_uavs
     if (nargin==1) 
      text_uav{i} = ['UAV ' num2str(i)];
     else 
      text_uav{i} = [text num2str(i)];
     end
   end
end