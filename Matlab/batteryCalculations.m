function [capacity, qty, mass] = batteryCalculations()
%   http://www.greencarcongress.com/2009/12/panasonic-20091225.html
%   http://www.roperld.com/science/TeslaModelS.htm
%   http://www.motortrend.com/news/2012-tesla-model-s-first-drive/
%   https://en.wikipedia.org/wiki/Tesla_Model_S#Battery

%% From USDE for nissan leaf
DC_RECHARGE_ENERGY_DELIVERED = 22.031*10^3;
DC_TEST_ENERGY              = 21.381*10^3;
BATTERY_EFFICIENCY          = DC_TEST_ENERGY/DC_RECHARGE_ENERGY_DELIVERED

%% Tesla (using the Li-ion Panasonic 18650 NCA cells)
E_cell      = 140;       	%W·h/kg
E_total     = 85*10^3;     	%Wh - ADVERTISED
M_total     = 540;       	% kg
N_modules   = 7104;        	%48 modules and each module contains four battery cells, a total of 192 cells

%%

M_cell = M_total/N_modules;
E_Cell = E_total/N_modules;
E_bat = E_Cell*3600*BATTERY_EFFICIENCY;

M_con = 0;
M_bat = M_cell;

%% POWER REQUIREMENTS
CAP         = 4.5e+06;
E           = 3600*CAP;
E           = 4.5e+06 + 1.1e+06 + 2e+06;
CAP         = (E/3600)/1000; %kwh


CAP         = 25e+03;	
E           = 3600*CAP;
CAP         = E/3600;

SAFETY_FACTOR = 1.5;

capacity    = E*SAFETY_FACTOR
qty         = capacity/E_bat
mass        = M_bat

CAP         = (E/3600)/1000 %kwh
MASS_TOTAL = (M_con + M_bat*qty)
end