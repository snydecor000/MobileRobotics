%% Load in data
load('TMap1.txt');
load('TMap2.txt');
load('OGrid.txt');
OGrid = ogrid2map(OGrid);
%% Visualize a map
map = TMap2;
fig = makeMapFigure(map);

%% Path planning
clear all;
clc;
load('TMap2.txt');
map = TMap2;

ans = findMetricPath(map,0,0,1,0);

function resultingPath = findMetricPath(map,startX,startY,endX,endY)
    [numRows,numCols] = size(map);
    startRow = numRows-startY;
    startCol = startX+1;
    endRow = numRows-endY;
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
