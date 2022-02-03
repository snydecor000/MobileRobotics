function map = ogrid2map(ogrid)
%OGRID2MAP Summary of this function goes here
%   Detailed explanation goes here
northWall = 1;
eastWall = 2;
southWall = 4;
westWall = 8;

numRows = size(ogrid,1);
numCols = size(ogrid,2);
map = zeros(numRows,numCols);

for r = 1:numRows
    for c = 1:numCols
        if(ogrid(r,c)==99)
            map(r,c) = 15;
        else
            % Check North Square
            if(r-1 < 1 || ogrid(r-1,c)==99)
                map(r,c) = map(r,c) + northWall;
            end
            % Check East Square
            if(c+1 > numCols || ogrid(r,c+1)==99)
                map(r,c) = map(r,c) + eastWall;
            end
            % Check South Square
            if(r+1 > numRows || ogrid(r+1,c)==99)
                map(r,c) = map(r,c) + southWall;
            end
            % Check West Square
            if(c-1 < 1 || ogrid(r,c-1)==99)
                map(r,c) = map(r,c) + westWall;
            end
        end
    end
end

end

