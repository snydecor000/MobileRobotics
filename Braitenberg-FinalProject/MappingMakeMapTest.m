map = zeros(4,4);
[numRows,numCols] = size(map);

mapFig = uifigure();
ax = uiaxes(mapFig);
axis(ax,[-1 numCols+1 -1 numRows+1])
rectangle(ax,'Position',[0 0 numCols numRows],'LineWidth',2);
% set(gcf,'position',[100,100,500,500]);

xLocNew = xLoc + 1;
yLocNew = numRows - yLoc;
map(xLocNew, yLocNew) = walls;

for r = 1:numRows
    for c = 1:numCols
        if(bitget(map(r,c),1)==1)
            x1 = c-1; x2 = c;
            y1 = numRows-(r-1); y2 = numRows-(r-1);
            line(ax,[x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
        if(bitget(map(r,c),2)==1)
            x1 = c; x2 = c;
            y1 = numRows-(r-1); y2 = numRows-r;
            line(ax,[x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
        if(bitget(map(r,c),3)==1)
            x1 = c-1; x2 = c;
            y1 = numRows-r; y2 = numRows-r;
            line(ax,[x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
        if(bitget(map(r,c),4)==1)
            x1 = c-1; x2 = c-1;
            y1 = numRows-(r-1); y2 = numRows-r;
            line(ax,[x1 x2],[y1 y2],'color','k','LineWidth',2)
        end
    end
end