
% FRICTION COEFFICIENTS
% Experimental coefficient of friction values

% Back friction = bf
% Forward friction = ff
% Side friction = sf

function [bf,ff,sf] = f_FricCoeff

% % Perfect Slipping
% bf = 0;
% ff = 0;
% sf = 0;
% % Ideal Lateral Undulation
% bf = 0;
% ff = 0;
% sf = 1;
% Almost Ideal Lateral Undulation
bf = 0.01;
ff = 0.01;
sf = 1;
% % Ideal Condition for Lateral + Transverse Undulation
% bf = 1;
% ff = 0;
% sf = 1;

% Coefficients for Various Materials
% (Measurements taken by Wael Saab and Anil  Kumar,
%  for Miniature Modular Inchworm Robot)

% % Anodized  Aluminum
% bf = 0.43;
% ff = 0.43;
% sf = 0.44;
% % Floor Tiles, New
% bf = 0.57;
% ff = 0.44;
% sf = 0.51;
% % Polished Wood
% bf = 0.68;
% ff = 0.44;
% sf = 0.48;
% % Corrugated Sheet
% bf = 1.463;
% ff = 0.506;
% sf = 0.973;
% % Composite Wood
%bf = 1.42;
%ff = 0.43;
%sf = 0.99;
% % Concrete Brick
%bf = 1.52;
%ff = 0.67;
%sf = 1.13;
%%%%%%%%%%%%%%%%%%%%%%
% % Carpet
% bf = 4.01;
% ff = 0.55;
% sf = 0.6; % ??
% Floor Tiles
% bf = 0.7813;
% ff = 0.4245;
% % Polished Wood
% bf = 0.8693;
% % Concrete Bar
% bf = 1.2799;
% ff = 0.9657;
% % Paper
% bf = 0.9325;
% ff = 0.404;
% % Cast Iron
% bf = 0.404;
% ff = 0.287;

end