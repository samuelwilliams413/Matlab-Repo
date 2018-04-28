function [capacity, qty, mass] = batteryCalculations()
%% From USDE
DC_RECHARGE_ENERGY_DELIVERED = 22.031*10^3;
DC_TEST_ENERGY              = 21.381*10^3;
BATTERY_EFFICIENCY          = DC_TEST_ENERGY/DC_RECHARGE_ENERGY_DELIVERED
%%
E_cell      = 140;       	%W·h/kg
E_total     = 24*10^3;     	%Wh - ADVERTISED
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
CAP     = 21.381*10^3;      %1.1*10^8
E       = 3600*CAP;         %1.1*10^8
E = 107918831.876071
E = 1.1 * 10^8
capacity    = E
qty         = E/E_bat
mass        = M_bat*qty + M_con

qty         = 0
mass        = 0
end