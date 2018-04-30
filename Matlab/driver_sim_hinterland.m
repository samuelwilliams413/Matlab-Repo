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

resolution = 50; % in meters

d_int = 0:resolution:L; % perform a calculation every 1m for the duration of the journey
e_int = interp1(d,e,d_int);
LATITUDES = interp1(d,LATITUDES_f,d_int);
LONGITUDES = interp1(d,LONGITUDES_f,d_int);

distance_cache          = zeros(length(d_int),1);
displacement_c                   = zeros(length(d_int),1);
gradient_cache          = zeros(length(d_int),1);
vc                      = zeros(length(d_int),1);
velocity_c          = zeros(length(d_int),1);
acceleration_c    	= zeros(length(d_int),1);
F_trac_cache            = zeros(length(d_int),1);
TL_cache                = zeros(length(d_int),1);
TL_MOTOR_cache          = zeros(length(d_int),1);
power_cache             = zeros(length(d_int),1);
work_cache              = zeros(length(d_int),1);
power_accumulator_cache = zeros(length(d_int),1);
time_a_c                 = zeros(length(d_int),1);
energy_cache            = zeros(length(d_int),1);
e_a_c                   = zeros(length(d_int),1);
bat_o                   = zeros(length(d_int),1);
bat_i                   = zeros(length(d_int),1);
bat_t                   = zeros(length(d_int),1);
f_store                 = zeros(length(d_int),4);
v_store                 = zeros(length(d_int),1);
u_store                 = zeros(length(d_int),1);

[capacity, qty, weight] = batteryCalculations()
setBATTERY_PARAMETERS(capacity, qty, weight)

length_of_d_int = length(d_int);

speeds = zeros(length_of_d_int,1);
km = 1000;

v = kmhr_to_ms(50);

for i = 1:1:length_of_d_int
    if(10*km > d_int(i))
        v = kmhr_to_ms(50);
    elseif(20*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(30*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(40*km > d_int(i))
        v = kmhr_to_ms(60);
    elseif(50*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(60*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(70*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(80*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(90*km > d_int(i))
        v = kmhr_to_ms(60);
    elseif(100*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(110*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(120*km > d_int(i))
        v = kmhr_to_ms(60);
    elseif(130*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(140*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(150*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(160*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(170*km > d_int(i))
        v = kmhr_to_ms(60);
    elseif(180*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(190*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(200*km > d_int(i))
        v = kmhr_to_ms(90);
    elseif(210*km > d_int(i))
        v = kmhr_to_ms(110);
    elseif(220*km > d_int(i))
        v = kmhr_to_ms(60);
    else
        v = kmhr_to_ms(50);
    end
    speeds(i) = v;
end

clc
resolution_in_meters = resolution
gear = 1;
for inc = 1:1:length(d_int)
    
    [d_i, e_i, lat, long] = get_DELL(d_int,e_int,LATITUDES,LONGITUDES, inc);
    if (inc == 1)
        u = speeds(1);
        dd = 0;
        de = 0;
        displacement_c(inc) = dd;
    else
        dd = d_i - d_o;
        de = e_i - e_o;
        displacement_c(inc) = dd + displacement_c((inc - 1));
    end
    ds = sqrt(dd^2 + de^2);
    current_distance = displacement_c(inc);
    
    %%
    
    v_desired = speeds(inc);
    
    %%
    a = 0;
    a_max = .5;
    if(u ~= v_desired)
        a = (v_desired^2 - u^2)/(2*ds);
        if (a > a_max)
            a = a_max;
        elseif (a < -a_max)
            a = -a_max;
        end
        v = sqrt(u^2 + 2*a*ds);
    end
    dv = v - u;
    dt = ds/u;
    %%
    distance_cache(inc) = ds;
    acceleration_c(inc) = a;
    velocity_c(inc) = v;
    
    if (inc == 1)
        time_a_c(inc) = dt;
    else
        time_a_c(inc) = dt + time_a_c((inc - 1));
    end
    [dt, a, v, ds, dv];
    
    u = v;
    d_o = d_i;
    e_o = e_i;
    
    %%
    
    m = get_gradient(dd, de);
    gradient_c(inc) = m;
    
    [F_trac] = get_F_trac(m, v, a);
    force_trac_c(inc) = F_trac;
    
    [TL_wheel] = get_TL_on_Wheels(F_trac);
    TL_c(inc) = TL_wheel;
    
    [TL_motor] = gear_transform_TL(gear, TL_wheel);
    TM_c(inc) = TL_motor;
    
    
    %% DOESN'T CHANGE
    
    %get work
    [W] = ds*F_trac;
    P = W/dt;
    
    if (P > 0)
        bat_o(inc) = P/c.MOTOR_OUTPUT_EFFICIENCY;
        bat_i(inc) = 0;
    else
        bat_o(inc) = 0;
        bat_i(inc) = (-P)*c.REGEN_BRAKING_EFFICIENCY;
    end
    
    if(inc ~= 1)
        bat_t(inc) = bat_t(inc-1) - bat_o(inc) + bat_i(inc);
    end
    
    if(inc == 1)
        bat_t(inc) = bat_t(inc) + getCAPACITY();
    end
    
end
%%
L
resolution          = resolution
extra_charge_needed = min(min(min(bat_t(:,:,:))))
extra_charge_needed = min(min(min(bat_t(:,:,:))))/1000/3600
capacity            = getCAPACITY()
qty                 = getQTY()
weight              = getWEIGHT()


close all
figure('Name',strcat('Simulation of UQ-Kingbeach-Maleny-UQ Drive (228 km)',int2str(resolution)));

y_plots = 2;
x_plots = 3;
count = 1;
time_unit = 'time (s)';

time = time_a_c(:);

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Elevation');
hold on
count = count +1;
y = e_int(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Elevation (m)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Displacement');
hold on
count = count +1;
y = displacement_c(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Distance (m)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Traction Force');
hold on
count = count +1;
y = force_trac_c(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'F_{trac} (N)')

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Velocity');
hold on
count = count +1;
y = 3.6*velocity_c(:);
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Velocity (kph)')

ax = subplot(x_plots,y_plots,count);
TITLE = strcat('Time vs Charge - Inital:',(int2str(capacity/1000/3600)),'kWH');
title(ax,TITLE);
hold on
count = count +1;
y = bat_t(:)/3600/1000;
plot(time(:), y(:))
xlabel(ax,time_unit)
ylabel(ax,'Charge (kWH)')
%axis([0 6000 0 50])

ax = subplot(x_plots,y_plots,count);
title(ax,'Time vs Acceleration');
hold on
count = count +1;
y = acceleration_c(:);
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

d_lat = abs(LATITUDES - lat);
d_lon = abs(LONGITUDES - long);

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