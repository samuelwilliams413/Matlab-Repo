classdef c
    properties (Constant)
        C_roll = 0.02; %--
        M_veh= 1600; %kg
        g = 9.81; %m/s^2
        Rho_air = 1.225; %kg/m^3
        A_f = 1.6; %m^2
        C_d= 0.26; %--
        rw = 15 % inches [15-19] need to be tested
        regen = 0 % This is the percentage of power we get back from regenerative braking
        filename = 'D:\Google Drive\Part B\Working Documents\Code\Matlab-Repo\Matlab\UQ_Kingsbeach_Maleny_UQ.csv';
        GEAR_RATIOS = [1,5,10,15];
        SPEED_LIMITS = [30,50,70,90,110];
        transmission_efficiency = 1;
        baseCharge = 5*10^6; % initial battery charge
        motor_output_efficiency = 1;
        regen_braking_efficiency = .55; %some random figure i found online
    end
end