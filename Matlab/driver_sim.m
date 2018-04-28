%% Initialisation
clc
clear
close all

PLOTTING = true;

import constants.*;


setFILE() % unless the folder 'constants' (and its subfolders) is added this step will fail
file = getFILE();
d = file(:,1);
e = file(:,2);
L = max(d);

resolution = 1000; % in meters

distance_interpolated = 0:resolution:L; % perform a calculation every 1m for the duration of the journey
elevation_interpolated = interp1(d,e,distance_interpolated);

%s = 10:10:110; %for all speed limits [10-110km/hr]
trials = length(c.SPEED_LIMITS);
gears = length(c.GEAR_RATIOS);

gradient_cache          = zeros(trials,gears,length(distance_interpolated));
F_trac_cache            = zeros(trials,gears,length(distance_interpolated));
TL_cache                = zeros(trials,gears,length(distance_interpolated));
TL_MOTOR_cache          = zeros(trials,gears,length(distance_interpolated));
power_cache             = zeros(trials,gears,length(distance_interpolated));
work_cache              = zeros(trials,gears,length(distance_interpolated));
power_accumulator_cache = zeros(trials,gears,length(distance_interpolated));
time_accumulator_cache  = zeros(trials,gears,length(distance_interpolated));
energy_cache            = zeros(trials,gears,length(distance_interpolated));
energy_accumulator_cache= zeros(trials,gears,length(distance_interpolated));
battery_out             = zeros(trials,gears,length(distance_interpolated));
battery_in              = zeros(trials,gears,length(distance_interpolated));
battery_charge          = zeros(trials,gears,length(distance_interpolated));

[capacity, qty, weight] = batteryCalculations()
setBATTERY_PARAMETERS(capacity, qty, weight)

%% Working
resolution_in_meters = resolution
speeds_to_be_tested = c.SPEED_LIMITS
gears_to_be_tested = c.GEAR_RATIOS

for t = 1:1:trials             %for all speed limits
    speed_limit = kmhr_to_ms(c.SPEED_LIMITS(t));
    
    for g = 1:1:gears   %for all gear ratios
        clc
        PROGRESS = [t,g]
        gear = c.GEAR_RATIOS(g);
        total_time = 0;
        total_power = 0;
        acceleration = 0;
        distance_now = 0;
        elevation_now = 0;
        gradient_now = 0;
        F_trac=0;
        power = 0;
        time_inc_old = 0;
        for inc = 1:1:length(distance_interpolated)          %every 10 m
            [dist_inc, elev_inc] = get_distance_and_elevation(distance_interpolated,elevation_interpolated, inc);
            time_inc = dist_speed_to_time(dist_inc, speed_limit);
            time_accumulator_cache(t,g,inc) = time_accumulator_cache(t,g,inc) + time_inc;
            
            if (inc == 1)
                delta_dist = 0;
                delta_elev = 0;
                delta_time = time_inc;
            else
                [dist_inc_old, elev_inc_old] = get_distance_and_elevation(distance_interpolated,elevation_interpolated, (inc-1));
                delta_dist = dist_inc - dist_inc_old;
                delta_elev = elev_inc - elev_inc_old;
                delta_time = time_inc - time_inc_old;
            end
            
            gradient = get_gradient(delta_dist, delta_elev);
            gradient_cache(t,g,inc) = gradient;
            
            % NO ACCELERATION CONSTANT SPEED
            % NO ACCELERATION CONSTANT SPEED
            % NO ACCELERATION CONSTANT SPEED
            [F_trac] = get_F_trac(gradient, speed_limit, 0);
            F_trac_cache(t,g,inc) = F_trac;
            
            [TL_wheel] = get_TL_on_Wheels(F_trac);
            TL_cache(t,g,inc) = TL_wheel;
            
            [TL_motor] = gear_transform_TL(gear, TL_wheel);
            TL_MOTOR_cache(t,g,inc) = TL_motor;
            
            [work_joules] = get_Work(delta_dist, delta_elev, F_trac);
            [work_WH] = joules_to_WH(work_joules);
            
            energy_motor = work_joules;
            energy_cache(t,g,inc) = energy_motor;
            
            if (energy_motor > 0)
                battery_out(t,g,inc) = energy_motor*c.MOTOR_OUTPUT_EFFICIENCY;
                battery_in(t,g,inc) = 0;
            else
                battery_out(t,g,inc) = 0;
                battery_in(t,g,inc) = (-energy_motor)*c.REGEN_BRAKING_EFFICIENCY;
            end
            
            
            if(inc ~= 1)
                battery_charge(t,g,inc) = battery_charge(t,g,inc-1) - battery_out(t,g,inc) + battery_in(t,g,inc);
            end
            
            if(inc == 1)
                battery_charge(t,g,inc) = battery_charge(t,g,inc) + getCAPACITY();
            end
            
            time_inc_old = time_inc;
        end
    end
end


resolution          = resolution
extra_charge_needed = min(min(min(battery_charge(:,:,:))))
capacity            = getCAPACITY()
qty                 = getQTY()
weight              = getWEIGHT()

if(PLOTTING == true)
    time_accumulator_cache = s_to_hr(time_accumulator_cache);
    time_unit = 'time(hours)';
    figure
    number_of_plots = 3;
    count = 1;
    grid_place = [1,5,9,2,6,10,3,7,11,4,8,12];
    
    for g = 1:1:gears
        gear = c.GEAR_RATIOS(g);
        ax1 = subplot(number_of_plots,gears,grid_place(count));
        count = count + 1;
        hold on
        
        for t = 1:1:trials
            time_accumulator_cache(t,g,1) = 0;
            
            time = time_accumulator_cache(t,g,:);
            y = elevation_interpolated(:);
            plot(time(:), y(:))
        end
        
        ax2 = subplot(number_of_plots,gears,grid_place(count));
        count = count + 1;
        hold on
        
        for t = 1:1:trials
            time_accumulator_cache(t,g,1) = 0;
            
            time = time_accumulator_cache(t,g,:);
            y = energy_cache(t,g,:);
            plot(time(:), y(:))
        end
        
        ax3 = subplot(number_of_plots,gears,grid_place(count));
        count = count + 1;
        hold on
        
        for t = 1:1:trials
            time_accumulator_cache(t,g,1) = 0;
            
            time = time_accumulator_cache(t,g,:);
            y = (1/1000)*battery_charge(t,g,:)/3600;
            y = battery_charge(t,g,:);
            plot(time(:), y(:))
        end
        
        stringtoprint = strcat('Elevation','[Gear Ratio: ',int2str(gear),']');
        title(ax1,stringtoprint)
        xlabel(ax1,time_unit)
        ylabel(ax1,'Elevation')
        legend(ax1,int2str(c.SPEED_LIMITS(1)),int2str(c.SPEED_LIMITS(2)),int2str(c.SPEED_LIMITS(3)),int2str(c.SPEED_LIMITS(4)),int2str(c.SPEED_LIMITS(5)));
        
        
        title(ax2,'Energy')
        xlabel(ax2,time_unit)
        ylabel(ax2,'Torque')
        legend(ax2,int2str(c.SPEED_LIMITS(1)),int2str(c.SPEED_LIMITS(2)),int2str(c.SPEED_LIMITS(3)),int2str(c.SPEED_LIMITS(4)),int2str(c.SPEED_LIMITS(5)));
        
        
        stringtoprint = strcat('Power{',(int2str(getCAPACITY())),'}');
        title(ax3,stringtoprint)
        xlabel(ax3,time_unit)
        ylabel(ax3,'Charge')
        legend(ax3,int2str(c.SPEED_LIMITS(1)),int2str(c.SPEED_LIMITS(2)),int2str(c.SPEED_LIMITS(3)),int2str(c.SPEED_LIMITS(4)),int2str(c.SPEED_LIMITS(5)));
        
    end
end

%% Function Definitions: File Manipulation

function [] = setFILE()
global FILE
FILE = csvread(c.FILENAME,1,0);
end

function [f] = getFILE()
global FILE
f = FILE;
end

%% Function Definitions: Battery Stuff

function [] = setBATTERY_PARAMETERS(capacity, qty, weight)
global QTY
global CAPACITY
global WEIGHT
QTY = qty;
CAPACITY = capacity;
WEIGHT = weight;
end

function [qty] = getQTY()
global QTY
qty = QTY;
end

function [capacity] = getCAPACITY()
global CAPACITY
capacity = CAPACITY;
end

function [weight] = getWEIGHT()
global WEIGHT
weight = WEIGHT;
end

%% Function Definitions: Conversions and Handlers

function [hr] = s_to_hr(s)
hr = s/3600;
end

function [WH] = joules_to_WH(J)
WH = J/3600;
end

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

function [gradient] = get_gradient(delta_dist, delta_elev)
%get_gradient: get the road gradient from distance and elevation
gradient = atand((delta_elev/delta_dist));

end

function m = m_v()
%get_gradient: get the road gradient from distance and elevation
m = getWEIGHT()*getQTY() + c.M_veh;
end

%% Function Definitions: FBD Model

function [F_roll] = get_F_roll(gradient) %Sigma in degrees
%get_F_roll: get F_roll
import constants.*;
F_roll = c.C_roll*m_v()*c.g*cosd(gradient);
end

function [F_aero] = get_F_aero(velocity)
%get_F_aero: get F_aero
import constants.*;
F_aero = (1/2)*c.Rho_air*c.A_f*c.C_d*velocity*velocity;
end

function [F_grade] = get_F_grade(gradient)
%get_F_grade: get F_grade
import constants.*;
F_grade = m_v()*c.g*sind(gradient);
end

function [F_inertia] = get_F_inertia(acceleration)
%get_inertial_force: get inertial force
import constants.*;
F_inertia = m_v()*acceleration;
end

function [F_trac] = get_F_trac(gradient, velocity, acceleration)
%get_F_trac: get traction force on external edge of wheels
F_trac = get_F_inertia(acceleration) + get_F_grade(gradient) + get_F_aero(velocity) + get_F_roll(gradient);
end

function [TL_wheel] = get_TL_on_Wheels(F_trac)
%get_TL_on_Wheels: get torque load on Wheels
import constants.*;
TL_wheel = inches_to_cm(c.RADIUS_OF_WHEEL)*F_trac;
end

function [W] = get_Work(delta_dist, delta_elev, F_trac)
import constants.*;
F = F_trac;
d = sqrt(delta_dist*delta_dist + delta_elev*delta_elev);
W = F*d;
end

function [TL_motor] = gear_transform_TL(gear, TL_wheel)
import constants.*;
TL_motor = TL_wheel/gear;
end