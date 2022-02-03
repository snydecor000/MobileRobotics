%% Load in data
load('TMap1.txt');
load('TMap2.txt');
load('OGrid.txt');
OGrid = ogrid2map(OGrid);
%% Visualize a map
map = TMap1;
fig = makeMapFigure(map);