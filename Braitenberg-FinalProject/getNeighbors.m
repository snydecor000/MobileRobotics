function neighbors = getNeighbors(map,r,c)
    neighbors=zeros(1,2);
    % Check North Square
    if(bitget(map(r,c),1)==0)
        neighbors(end+1,:) = [r-1 c];
    end
    % Check East Square
    if(bitget(map(r,c),2)==0)
        neighbors(end+1,:) = [r c+1];
    end
    % Check South Square
    if(bitget(map(r,c),3)==0)
        neighbors(end+1,:) = [r+1 c];
    end
    % Check West Square
    if(bitget(map(r,c),4)==0)
        neighbors(end+1,:) = [r c-1];
    end
    neighbors(1,:) = [];
end