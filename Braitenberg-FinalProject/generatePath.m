function path = generatePath(BFSresult,startR,startC)
    [numRows,numCols] = size(BFSresult);
    r = startR;
    c = startC;
    path(1) = 'Q';
    while(BFSresult(r,c) ~= 0)
        % Check North Square
        if(r-1 >= 1 && BFSresult(r-1,c)==BFSresult(r,c)-1)
            path(end+1) = 'N';
            r = r-1;
        % Check East Square
        elseif(c+1 <= numCols && BFSresult(r,c+1)==BFSresult(r,c)-1)
            path(end+1) = 'E';
            c = c+1;
        % Check South Square
        elseif(r+1 <= numRows && BFSresult(r+1,c)==BFSresult(r,c)-1)
            path(end+1) = 'S';
            r = r+1;
        % Check West Square
        elseif(c-1 >= 1 && BFSresult(r,c-1)==BFSresult(r,c)-1)
            path(end+1) = 'W';
            c = c-1;
        end
    end
    path(1) = [];
end