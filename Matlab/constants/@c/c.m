classdef c
    properties (Constant)
        FILENAME = 'D:\Google Drive\Part B\Working Documents\Code\Matlab-Repo\Matlab\UQ_Kingbeach_Maleny_UQ_withLATLONG.csv';
        
        C_roll = 0.02; %--
        M_veh= 1200; %kg
        g = 9.81; %m/s^2
        Rho_air = 1.225; %kg/m^3
        A_f = 1.6; %m^2
        C_d= 0.26; %--
        
        RADIUS_OF_WHEEL = 15 % inches [15-19] need to be tested
        
        GEAR_RATIOS = [1,5,10,15];
        SPEED_LIMITS = [30,50,70,90,110];
        
        TRANSMISSION_EFFICIENCY = 1;
        MOTOR_OUTPUT_EFFICIENCY = 1;
        
        INITIAL_BATTERY_CHARGE = 5*10^6; % initial battery charge
        BATTERY_WEIGHT = 400;
        BATTERY_CAPACITY = 5*10^6;
        
        REGEN_BRAKING_EFFICIENCY = 0; %some random figure i found online
    end
end
