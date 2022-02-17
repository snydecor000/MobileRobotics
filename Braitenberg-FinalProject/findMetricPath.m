function resultingPath = findMetricPath(map,startX,startY,endX,endY)
    startX = str2num(cell2mat(startX));
    startY = str2num(cell2mat(startY));
    endX = str2num(cell2mat(endX));
    endY = str2num(cell2mat(endY));
    [numRows,numCols] = size(map);
    startRow = numRows - startY;
    startCol = startX+1;
    endRow = numRows - endY;
    endCol = endX+1;

    visited = zeros(size(map));
    visited(endRow,endCol) = 1;

    queue(1,:) = [endRow endCol];
    result = ones(size(map))*99;
    result(endRow,endCol) = 0;

    while(queue(1,1) ~= startRow || queue(1,2) ~= startCol)
        current = queue(1,:);
        queue(1,:) = [];
        r = current(1);
        c = current(2);
        visited(r,c) = 1;

        n = getNeighbors(map,r,c);
        if(size(n,1)>0)
            for i = 1:size(n,1)
                nr = n(i,1);
                nc = n(i,2);
                if(visited(nr,nc)==0)
                    visited(nr,nc) = 1;
                    result(nr,nc) = result(r,c)+1;
                    queue(end+1,:) = [nr nc];
                end
            end
        end
    end

    resultingPath = generatePath(result,startRow,startCol);
end
