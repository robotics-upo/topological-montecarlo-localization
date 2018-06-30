
function load_stats_localization(name, first, last, title_)
    data = zeros(2,2);
    for i_=first:last
        i=i_-first+1
        realname = [name num2str(i_) '.txt'];
        s = load(realname);
        
        if (length(data)== 2)
            data = zeros(length(s)/3, first-last);
        end
        
        for j = 1:3:length(s)
            
	    row = (j+2)/3;
	    dist = min(s(j:j+2,7))
            data(row,i)=dist;
        end
    end

%      boxplot(data);
    h = figure;
    boxplot(data');
    figure(h);
    setLabelStyle('Visited Manhole number', 'Localization error (m)');
    if (nargin > 3)
      title(title_);
    end
end