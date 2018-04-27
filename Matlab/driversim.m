%% Initialisation
close all
import constants.*;
setFILE() % unless the folder 'constants' (and its subfolders) is added this step will fail
file = getFILE();

d = file(:,1);
e = file(:,2);
L = max(d);
resolution = 1; % in meters
distance_interpolated = 0:resolution:L; % perform a calculation every 10m for the duration of the journey
elevation_interpolated = interp1(d,e,distance_interpolated);

s = 10:10:110; %for all speed limits [10-110km/hr]
%s = [50,80,110];

trials = length(s);
gears = length(c.GEAR_RATIOS);

F_trac_cache            = zeros(trials,gears,length(distance_interpolated));
TL_cache                = zeros(trials,gears,length(distance_interpolated));
TL_MOTOR_cache          = zeros(trials,gears,length(distance_interpolated));
power_cache             = zeros(trials,gears,length(distance_interpolated));
work_cache              = zeros(trials,gears,length(distance_interpolated));
power_accumulator_cache = zeros(trials,gears,length(distance_interpolated));
time_accumulator_cache  = zeros(trials,gears,length(distance_interpolated));

battery_out             = zeros(trials,gears,length(distance_interpolated));
battery_in              = zeros(trials,gears,length(distance_interpolated));
battery_charge          = zeros(trials,gears,length(distance_interpolated));

%% Working
speeds_to_be_tested = s
gears_to_be_tested = c.GEAR_RATIOS
for t = 1:1:trials             %for all speed limits
    speed_limit = kmhr_to_ms(s(t))
    
    V = speed_limit;
    for g = 1:1:gears   %for all gear ratios
        gear = c.GEAR_RATIOS(g);
        total_time = 0;
        total_power = 0;
        acceleration = 0;
        distance_now = 0;
        elevation_now = 0;
        gradient_now = 0;
        F_trac=0;
        power = 0;
        for incriment = 1:1:length(distance_interpolated)          %every 10 m
            
            
            % Save last parameters
            distance_last = distance_now;
            elevation_last = elevation_now;
            gradient_last = gradient_now;
            F_trac_last = F_trac;
            
            %update distance and elevation
            [distance_now,elevation_now] = get_distance_and_elevation(distance_interpolated,elevation_interpolated, incriment);
            delta_distance = distance_now - distance_last;
            delta_elevation = elevation_now - elevation_last;
            
            %update gradient
            gradient_now = get_gradient(delta_distance,delta_elevation);
            delta_gradient = gradient_now - gradient_last;
            
            %get true distance travels
            delta_displacement = sqrt((delta_distance*delta_distance) + (delta_elevation*delta_elevation));
            
            %time incriment
            delta_t = dist_speed_to_time(speed_limit,delta_displacement);
            
            sigma = gradient_now;
            
            F_trac = get_F_trac(sigma, V, acceleration);
            delta_F_trac = F_trac - F_trac_last;
            F_trac_cache(t,g,incriment) = F_trac;
            
            [TL_wheel] = get_TL_on_Wheels(F_trac);
            TL_cache(t,g,incriment) = TL_wheel;
            
            TL_MOTOR_cache(t,g,incriment) = gear_transform_TL(gear, TL_wheel);
            
            work_cache(t,g,incriment) = get_Work(delta_displacement, F_trac);
            
            
            [power] = F_trac*speed_limit;
            
            power_cache(t,g,incriment) = power;
            
            %if power < 0 % We currently have 0% regenerative braking
            %    power = c.regen*power;
            %end
            
            total_time = delta_t + total_time;
            total_power = power + total_power;
            
            
            power_accumulator_cache(t,g,incriment) = total_power;
            time_accumulator_cache(t,g,incriment) = total_time;
            
        end
    end
end


F_trac_cache(1,1,1) = 0;
F_trac_cache(2,1,1) = 0;
F_trac_cache(3,1,1) = 0;
TL_cache(1,1,1) = 0;
TL_cache(2,1,1) = 0;
TL_cache(3,1,1) = 0;
TL_MOTOR_cache(1,1,1) = 0;
TL_MOTOR_cache(2,1,1) = 0;
TL_MOTOR_cache(3,1,1) = 0;

work_cache(1,1,1) = 0;
work_cache(2,1,1) = 0;
work_cache(3,1,1) = 0;
power_cache(1,1,1) = 0;
power_cache(2,1,1) = 0;
power_cache(3,1,1) = 0;
power_accumulator_cache;


figure
count = 1;
for g = 1:1:gears
    gear = c.GEAR_RATIOS(g);
    ax1 = subplot(gears,3,count);
    count = count + 1;
    hold on
    
    for t = 1:1:trials
        time_accumulator_cache(t,g,1) = 0;
        
        time = time_accumulator_cache(t,g,:);
        y = power_accumulator_cache(t,g,:);
        plot(time(:), y(:))
    end
    
    ax2 = subplot(gears,3,count);
    count = count + 1;
    hold on
    
    for t = 1:1:trials
        time_accumulator_cache(t,g,1) = 0;
        
        time = time_accumulator_cache(t,g,:);
        y = TL_MOTOR_cache(t,g,:);
        plot(time(:), y(:))
    end
    
    ax3 = subplot(gears,3,count);
    count = count + 1;
    hold on
    
    for t = 1:1:trials
        time_accumulator_cache(t,g,1) = 0;
        
        time = time_accumulator_cache(t,g,:);
        y = F_trac_cache(t,g,:);
        plot(time(:), y(:))
    end
    
    
    title(ax1,'power\_accumulated')
    xlabel(ax1,'time(s)')
    ylabel(ax1,'power\_accumulated')
    legend(ax1,int2str(s(1)),int2str(s(2)),int2str(s(2)));
    
    stringtoprint = strcat('Gear Ratio: ',int2str(gear));
    title(ax2,stringtoprint)
    xlabel(ax2,'time(s)')
    ylabel(ax2,'TL_MOTOR_cache')
    legend(ax1,int2str(s(1)),int2str(s(2)),int2str(s(2)));
    
    title(ax3,'F\_trac')
    xlabel(ax3,'time(s)')
    ylabel(ax3,'F\_trac')
    legend(ax1,int2str(s(1)),int2str(s(2)),int2str(s(2)));
end





%% Function Definitions: File Manipulation

function [] = setFILE()
global FILE
FILE = csvread(c.filename,1,0);
end

function [f] = getFILE()
global FILE
f = FILE;
end

%% Function Definitions: Conversions and Handlers

function [time] = dist_speed_to_time(distance, speed)
time = distance/speed;
end

function [velocity_ms] = kmhr_to_ms(velocity_kmhr)
velocity_ms = velocity_kmhr/3.6;
end

function [cm] = inches_to_cm(inches)
cm = 2.54*inches;
end

function [d, e] = get_distance_and_elevation(distance_interpolated,elevation_interpolated, index)
%get_distance_and_elevation: get distance and elevation
import constants.*;
d = distance_interpolated(index);
e = elevation_interpolated(index);
end

function [gradient] = get_gradient(delta_distance, delta_elevation)
%get_gradient: get the road gradient from distance and elevation
gradient = atand((delta_elevation/delta_distance));
end

%% Function Definitions: FBD Model

function [F_roll] = get_F_roll(sigma) %Sigma in degrees
%get_F_roll: get F_roll
import constants.*;
F_roll = c.C_roll*c.M_veh*c.g*cosd(sigma);
end

function [F_aero] = get_F_aero(V)
%get_F_aero: get F_aero
import constants.*;
F_aero = (1/2)*c.Rho_air*c.A_f*c.C_d*V*V;
end

function [F_grade] = get_F_grade(sigma)
%get_F_grade: get F_grade
import constants.*;
F_grade = c.M_veh*c.g*sind(sigma);
end

function [F_inertia] = get_F_inertia(a)
%get_inertial_force: get inertial force
import constants.*;
F_inertia = c.M_veh*a;
end

function [F_trac] = get_F_trac(sigma, V, a)
%get_F_trac: get traction force on external edge of wheels
F_trac = get_F_inertia(a) + get_F_grade(sigma) + get_F_aero(V) + get_F_roll(sigma);
end

function [TL_wheel] = get_TL_on_Wheels(F_trac)
%get_TL_on_Wheels: get torque load on Wheels
import constants.*;
TL_wheel = inches_to_cm(c.rw)*F_trac;
end

function [work] = get_Work(displacement, F_trac)
import constants.*;
work = displacement*F_trac;
end

function [power] = get_Power(displacement, t, F_trac)
import constants.*;
power = (displacement/t)*F_trac;
end

function [TL_motor] = gear_transform_TL(gear, TL_wheel)
import constants.*;
TL_motor = TL_wheel/gear;
end

function [TL_motor] = gear_transform_W(gear, speed)
import constants.*;
TL_motor = speed*gear;
end