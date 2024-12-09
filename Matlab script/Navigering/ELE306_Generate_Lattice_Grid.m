

function out=ELE306_Generate_Lattice_Grid(occupancyGrid,gridFactor,inflationFactor,gridRoot,description1)

% Denne funksjonen genererer et Lattice grid for bruk av en lattice planner
% som en del av navigasjonsstrategien til en mobil robot. 
% Argumenter er occupancy griddet, samt argumenter for generering av
% lattice grid. Justering av gridoppløsning, definering av start punkt for
% beregning av lattice og oppblåsning av hindringer.
%Kostnad er default i funksjonen hvor det er satt høy kost for sving.


latticeGrid = Lattice(occupancyGrid,'grid',gridFactor,'root',gridRoot,'inflate', inflationFactor, 'cost',[1 30 30]); 
latticeGrid.plan() 
figure; 
latticeGrid.plot();
yaxis(size(occupancyGrid,1));
title(description1);
out=latticeGrid;