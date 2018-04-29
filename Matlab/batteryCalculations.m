function [capacity, qty, mass] = batteryCalculations()
%% From USDE
DC_RECHARGE_ENERGY_DELIVERED = 22.031*10^3;
DC_TEST_ENERGY              = 21.381*10^3;
BATTERY_EFFICIENCY          = DC_TEST_ENERGY/DC_RECHARGE_ENERGY_DELIVERED
%% Nissan Leaf
E_cell      = 140;       	%W�h/kg
E_total     = 24*10^3;     	%Wh - ADVERTISED85 kWh
E_real      = 21.3*10^3;   	%Wh - MEASURE
M_total     = 218;       	% kg
N_modules   = 48;        	%48 modules and each module contains four battery cells, a total of 192 cells

%% Tesla (using the Li-ion Panasonic 18650 NCA cells)
E_cell      = 140;       	%W�h/kg
E_total     = 85*10^3;     	%Wh - ADVERTISED
E_real      = 21.3*10^3;   	%Wh - MEASURE
M_total     = 218;       	% kg
N_modules   = 48;        	%48 modules and each module contains four battery cells, a total of 192 cells


%%
M_cells = E_total/E_cell;
M_con   = M_total - M_cells;

M_bat   = M_cells/N_modules;
E_bat   = E_real/N_modules;

M_bat   = M_bat;
E_bat   = E_bat*3600;
%% POWER REQUIREMENTS
CAP         = 21.381*10^3	
E           = 3600*CAP
CAP         = 0
E           = 3600*CAP
CAP         = E/3600

capacity    = E
qty         = E/E_bat
mass        = M_bat*qty + M_con



%% Tesla (using the Li-ion Panasonic 18650 NCA cells)
clc
E_cell      = 140;       	%W�h/kg
E_total     = 85*10^3;     	%Wh - ADVERTISED
M_total     = 540;       	% kg
N_modules   = 7104;        	%48 modules and each module contains four battery cells, a total of 192 cells

%%

M_cell = M_total/N_modules
E_Cell = E_total/N_modules
E_bat = E_Cell*3600*BATTERY_EFFICIENCY

M_con = 0
M_bat = M_cell

%% POWER REQUIREMENTS
CAP         = 21.381*10^3	
E           = 3600*CAP
capacity    = E
qty         = E/E_bat
mass        = M_bat*qty + M_con
end