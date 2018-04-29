%% Initialisation
clc
clear
close all

PLOTTING = false;

import constants.*;


setFILE() % unless the folder 'constants' (and its subfolders) is added this step will fail
file = getFILE();
d = file(:,1);
e = file(:,2);
LATITUDES_f = file(:,3);
LONGITUDES_f = file(:,4);
L = max(d);

file_gov = csvread("D:\Google Drive\Part B\Working Documents\Code\Matlab-Repo\Matlab\SEQ_roads_lat_long_scraped.csv",1,0);
LATITUDES_gov = file_gov(:,2);
LONGITUDES_gov = file_gov(:,3);
LIMITS_gov = file_gov(:,1);

resolution = 1000; % in meters

d_int = 0:resolution:L; % perform a calculation every 1m for the duration of the journey
e_int = interp1(d,e,d_int);
LATITUDES = interp1(d,LATITUDES_f,d_int);
LONGITUDES = interp1(d,LONGITUDES_f,d_int);



%s = 10:10:110; %for all speed limits [10-110km/hr]
trials = 1;
gears = 1;

distance_cache          = zeros(trials,gears,length(d_int));
d_a_c                   = zeros(trials,gears,length(d_int));
gradient_cache         = zeros(trials,gears,length(d_int));
vc                      = zeros(trials,gears,length(d_int));
velocity_cache          = zeros(trials,gears,length(d_int));
acceleration_cache    	= zeros(trials,gears,length(d_int));
F_trac_cache            = zeros(trials,gears,length(d_int));
TL_cache                = zeros(trials,gears,length(d_int));
TL_MOTOR_cache          = zeros(trials,gears,length(d_int));
power_cache             = zeros(trials,gears,length(d_int));
work_cache              = zeros(trials,gears,length(d_int));
power_accumulator_cache = zeros(trials,gears,length(d_int));
t_cache                 = zeros(trials,gears,length(d_int));
energy_cache            = zeros(trials,gears,length(d_int));
e_a_c                   = zeros(trials,gears,length(d_int));
bat_o                   = zeros(trials,gears,length(d_int));
bat_i                   = zeros(trials,gears,length(d_int));
bat_t                   = zeros(trials,gears,length(d_int));

[capacity, qty, weight] = batteryCalculations()
setBATTERY_PARAMETERS(capacity, qty, weight)
%% velocity smoothing
building_speed_limits = true
for trial = 1:1:trials             %for all speed limits
    for g = 1:1:gears   %for all gear ratios
        time_inc_old = 0;
        for inc = 1:1:length(d_int)          %every 10 m
            [d_i, e_i, lat, long] = get_DELL(d_int,e_int,LATITUDES,LONGITUDES, inc);
            [minimum_distance, index] = getIndex(LATITUDES_gov,LONGITUDES_gov,lat, long);
            v = LIMITS_gov(index);
            vc(trial,g,inc) = kmhr_to_ms(v);
        end
    end
end

for s = 1:1:1
    smoothing = s
    vc = smooth(vc);
end

%% Working
resolution_in_meters = resolution
speeds_to_be_tested = c.SPEED_LIMITS
gears_to_be_tested = c.GEAR_RATIOS

for trial = 1:1:trials             %for all speed limits
    speed_limit = kmhr_to_ms(c.SPEED_LIMITS(trial));
    speed_limit = kmhr_to_ms(50);
    
    for g = 1:1:gears   %for all gear ratios
        clc
        PROGRESS = [trial,g]
        gear = c.GEAR_RATIOS(g);
        total_time = 0;
        total_power = 0;
        a = 0;
        distance_now = 0;
        elevation_now = 0;
        gradient_now = 0;
        F_trac=0;
        power = 0;
        time_inc_old = 0;
        for inc = 1:1:length(d_int)          %every 10 m
            [d_i, e_i, lat, long] = get_DELL(d_int,e_int,LATITUDES,LONGITUDES, inc);
            [minimum_distance, index] = getIndex(LATITUDES_gov,LONGITUDES_gov,lat, long);
            v = vc(inc);
            
            if (inc == 1)
                dd = d_i;
                de = e_i;
                dv = v;
                u = v;
                
                ds = sqrt(dd^2 + de^2);
                t_i = dist_speed_to_time(ds, u);
                dt = t_i;
                
                d_o = d_i;
                e_o = e_i;
                t_cache(trial,g,inc) = dt;
                d_a_c(trial,g,inc) = dd;
            else
                dd = d_i - d_o;
                de = e_i - e_o;
                dv = v - u;
                
                ds = sqrt(dd^2 + de^2);
                t_i = dist_speed_to_time(ds, u);
                dt = t_i;
                
                u = v;
                
                d_o = d_i;
                e_o = e_i;
                t_cache(trial,g,inc) = dt + t_cache(trial,g,(inc - 1));
                d_a_c(trial,g,inc) = dd + d_a_c(trial,g,(inc - 1));
            end
            
            a = dv/dt;
            
            distance_cache(trial,g,inc) = ds;
            acceleration_cache(trial,g,inc) = a;
            velocity_cache(trial,g,inc) = v;
            
            
            
            %%
            
            m = get_gradient(dd, de);
            gradient_c(trial,g,inc) = m;
            
            [F_trac] = get_F_trac(m, v, a);
            force_trac_c(trial,g,inc) = F_trac;
            
            [TL_wheel] = get_TL_on_Wheels(F_trac);
            TL_c(trial,g,inc) = TL_wheel;
            
            [TL_motor] = gear_transform_TL(gear, TL_wheel);
            TM_c(trial,g,inc) = TL_motor;
            
            
            %% DOESN'T CHANGE
            
            %get work
            [W] = ds*F_trac;
            P = W/dt;
            
            if (P > 0)
                bat_o(trial,g,inc) = P/c.MOTOR_OUTPUT_EFFICIENCY;
                bat_i(trial,g,inc) = 0;
            else
                bat_o(trial,g,inc) = 0;
                bat_i(trial,g,inc) = (-P)*c.REGEN_BRAKING_EFFICIENCY;
            end
            
            if(inc ~= 1)
                bat_t(trial,g,inc) = bat_t(trial,g,inc-1) - bat_o(trial,g,inc) + bat_i(trial,g,inc);
            end
            
            if(inc == 1)
                bat_t(trial,g,inc) = bat_t(trial,g,inc) + getCAPACITY();
            end
        end
    end
end
d_int = d_int(3:end);
e_int = e_int(3:end);
t_cache = t_cache(3:end);
distance_cache = distance_cache(3:end);
d_a_c = d_a_c(3:end);
velocity_cache = velocity_cache(3:end);
acceleration_cache = acceleration_cache(3:end);
force_trac_c = force_trac_c(3:end);
bat_t = bat_t(3:end);

resolution          = resolution
extra_charge_needed = min(min(min(bat_t(:,:,:))))
extra_charge_needed = min(min(min(bat_t(:,:,:))))/3600/1000
capacity            = getCAPACITY()
qty                 = getQTY()
weight              = getWEIGHT()

close all
figure('Name',strcat('Simulation of UQ-Kingbeach-Maleny-UQ Drive (228 km)',int2str(resolution)));

y_plots = 2;
x_plots = 3;
count = 1;
time_unit = 'time (s)';

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Elevation');
hold on
count = count +1;
time = t_cache(:);
y = e_int(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Elevation (m)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Displacement');
hold on
count = count +1;
time = t_cache(:);
y = distance_cache(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Distance (m)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Traction Force');
hold on
count = count +1;
time = t_cache(:);
y = force_trac_c(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'F_{trac} (N)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Velocity');
hold on
count = count +1;
time = t_cache(:);
y = 3.6*velocity_cache(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Velocity (kph)')

ax = subplot(x_plots,y_plots,count);
TITLE = strcat('Time vs Charge - Inital:',(int2str(capacity/1000/3600)),'kWH');
title(ax,TITLE);
hold on
count = count +1;
time = t_cache(:);
y = bat_t(:)/3600/1000;
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Charge (kWH)')
%axis([0 14000 0 50])

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Acceleration');
hold on
count = count +1;
time = t_cache(:);
y = acceleration_cache(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Acceleration (m/s^2)')


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
distance = distance;
speed = speed;
time = distance/speed;
end

function [velocity_ms] = kmhr_to_ms(velocity_kmhr)
velocity_ms = velocity_kmhr/3.6;
end

function [cm] = inches_to_cm(inches)
cm = 2.54*inches;
end

function [dist_inc, elev_inc, lat, long] = get_DELL(distance_interpolated,elevation_interpolated,LATITUDES,LONGITUDES, index)
%get_distance_and_elevation: get distance and elevation
import constants.*;
dist_inc = distance_interpolated(index);
elev_inc = elevation_interpolated(index);
lat = LATITUDES(index);
long = LONGITUDES(index);
end

function [gradient] = get_gradient(delta_dist, delta_elev)
%get_gradient: get the road gradient from distance and elevation
gradient = atand((delta_elev/delta_dist));

end

function m = m_v()
%get_gradient: get the road gradient from distance and elevation
m = getWEIGHT()*getQTY() + c.M_veh;
end

%% Speed Limits

function [minimum_distance, index] = getIndex(LATITUDES,LONGITUDES,lat, long)

d_lat = abs(LATITUDES - lat).^2;
d_lon = abs(LONGITUDES - long).^2;

[minimum_distance, index] = min(d_lat + d_lon);

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