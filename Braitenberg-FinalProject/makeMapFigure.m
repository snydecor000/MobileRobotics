function mapFig = makeMapFigure(map)
%MAKEMAPFIGURE Summary of this function goes here
%   Detailed explanation goes here
[numRows,numCols] = size(map);
mapFig = figure(4321);
hold on;
grid on;
axis([-1 numCols+1 -1 numRows+1])
rectangle('Position',[0 0 numCols numRows],'LineWidth',2);
% set(gcf,'position',[100,100,500,500]);
for r = 1:numRows
    for c = 1:numCols
        if(bitget(map(r,c),1)==1)
            x1 = c-1; x2 = c;
            y1 = numRows-(r-1); y2 = numRows-(r-1);
            line([x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
        if(bitget(map(r,c),2)==1)
            x1 = c; x2 = c;
            y1 = numRows-(r-1); y2 = numRows-r;
            line([x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
    end
end
hold off;
end

