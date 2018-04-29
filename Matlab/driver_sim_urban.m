%% Initialisation
clc
clear
close all

PLOTTING = false;

import constants.*;


setFILE() % unless the folder 'constants' (and its subfolders) is added this step will fail
file = getFILE();
s_v_f   = file(:,1);
e_v_f  	= file(:,2);
a_f   	= file(:,3);
d_f 	= file(:,4);
L = sum(d_f);

d_i(1) = d_f(1);
for segment = 2:1:length(d_f)
    d_i(segment) = d_f(segment) + d_i(segment-1);
    L = d_i(segment);
end

L


resolution =1; % in seconds

t_i = 0:resolution:L;

s_v_i  	= interp1(d_i,  s_v_f, 	t_i);
e_v_i   = interp1(d_i,  e_v_f, 	t_i);


count = 1;
for i = 1:1:length(t_i)
    current_time = t_i(i);
    a = 0;
    v = 0;
    
    for ii = 2:1:length(d_i)
        if (current_time > d_i(ii))
            a = a_f(ii);
            
        end
    end
    
    accl_i(i) = a;
    
end

time_incriment = 1/resolution;

a_c = accl_i;



for segment = 2:1:length(t_i)
    if (isnan(accl_i(segment)))
        accl_i(segment) = 0;
    end
    if (isnan(s_v_i(segment)))
        s_v_i(segment) = 0;
    end
    if (isnan(e_v_i(segment)))
        e_v_i(segment) = 0;
    end
end

for i = 1:time_incriment:length(accl_i)
    
    if ((i + d_f(1)/resolution) > length(e_v_i))
        a_c(i) = 0
    else
        a_c(i) = accl_i(i + d_f(1)/resolution);
    end
end

number_of_readings = length(t_i);

%s = 10:10:110; %for all speed limits [10-110km/hr]
trials  = length(c.SPEED_LIMITS);
gears   = length(c.GEAR_RATIOS);
trials = 1


time_a_c   = zeros(trials,gears,number_of_readings);
displacement_c   = zeros(trials,gears,number_of_readings);
velocity_c     = zeros(trials,gears,number_of_readings);
acceleration_c   = zeros(trials,gears,number_of_readings);
gradient_c     = zeros(trials,gears,number_of_readings);
force_trac_c     = zeros(trials,gears,number_of_readings);
TL_c   = zeros(trials,gears,number_of_readings);
TM_c   = zeros(trials,gears,number_of_readings);
p_c     = zeros(trials,gears,number_of_readings);
work_c     = zeros(trials,gears,number_of_readings);
p_a_c   = zeros(trials,gears,number_of_readings);
energy_c     = zeros(trials,gears,number_of_readings);
energy_a_c   = zeros(trials,gears,number_of_readings);
bat_out     = zeros(trials,gears,number_of_readings);
bat_in     = zeros(trials,gears,number_of_readings);
bat_t     = zeros(trials,gears,number_of_readings);

[capacity, qty, weight] = batteryCalculations()
setBATTERY_PARAMETERS(capacity, qty, weight)

%% Working
resolution_in_meters    = resolution
speeds_to_be_tested     = c.SPEED_LIMITS
gears_to_be_tested      = c.GEAR_RATIOS

for trial = 1:1:trials             %for all speed limits
    speed_limit = kmhr_to_ms(c.SPEED_LIMITS(trial));
    speed_limit = kmhr_to_ms(50);
    
    for g = 1:1:1   %for all gear ratios
        clc
        PROGRESS = [trial,g]
        
        gear = c.GEAR_RATIOS(g);
        total_time = 0;
        total_power = 0;
        distance_now = 0;
        elevation_now = 0;
        gradient_now = 0;
        F_trac=0;
        power = 0;
        t_old = 0;
        u = 0;
        de = 0;
        
        for inc = 1:1:number_of_readings
            %%
            de = 0;
            t = getDuration(t_i, inc);
            a = getAcceleration(a_c, inc);
            v = getVelocity(kmhr_to_ms(e_v_i), inc);
            
            if (inc == 1)
                dt = t;
                t_old = t;
            else
                dt = t - t_old;
                t_old = t;
            end
            
            dx = u*dt + (1/2)*a*dt*dt;
            u = v;
            
            time_a_c(trial,g,inc) = t;
            acceleration_c(trial,g,inc) = a;
            velocity_c(trial,g,inc) = v;
            
            if (inc == 1)
                displacement_c(trial,g,inc) = dx;
            else
                displacement_c(trial,g,inc) = dx + displacement_c(trial,g,(inc-1));
            end
            
            m = 0;
            gradient_c(trial,g,inc) = m;
            
            [F_trac] = get_F_trac(m, v, a);
            force_trac_c(trial,g,inc) = F_trac;
            
            [TL_wheel] = get_TL_on_Wheels(F_trac);
            TL_c(trial,g,inc) = TL_wheel;
            
            [TL_motor] = gear_transform_TL(gear, TL_wheel);
            TM_c(trial,g,inc) = TL_motor;
            
            [W_J] = dx*F_trac;
            [W_WH] = joules_to_WH(W_J);
            
            
            %% DOESN'T CHANGE
            E_M = W_J;
            energy_c(trial,g,inc) = dx;
            
            if (E_M > 0)
                bat_out(trial,g,inc) = E_M*c.MOTOR_OUTPUT_EFFICIENCY;
                bat_in(trial,g,inc) = 0;
            else
                bat_out(trial,g,inc) = 0;
                bat_in(trial,g,inc) = (-E_M)*c.REGEN_BRAKING_EFFICIENCY;
            end
            
            
            if(inc ~= 1)
                bat_t(trial,g,inc) = bat_t(trial,g,inc-1) - bat_out(trial,g,inc) + bat_in(trial,g,inc);
            end
            
            if(inc == 1)
                bat_t(trial,g,inc) = bat_t(trial,g,inc) + getCAPACITY();
            end
            
            t_old = t;
        end
    end
end







%%
L
resolution          = resolution
extra_charge_needed = min(min(min(bat_t(:,:,:))))
capacity            = getCAPACITY()
qty                 = getQTY()
weight              = getWEIGHT()

if(PLOTTING == true)
    time_a_c = s_to_hr(time_a_c);
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
        
        for trial = 1:1:trials
            time_a_c(trial,g,1) = 0;
            
            time = time_a_c(trial,g,:);
            y = accl_i(:);
            plot(time(:), y(:))
        end
        
        ax2 = subplot(number_of_plots,gears,grid_place(count));
        count = count + 1;
        hold on
        
        for trial = 1:1:trials
            time_a_c(trial,g,1) = 0;
            
            time = time_a_c(trial,g,:);
            %y = s_v_i(t,g,:);
            y = velocity_c(trial,g,:);
            plot(time(:), y(:))
        end
        
        ax3 = subplot(number_of_plots,gears,grid_place(count));
        count = count + 1;
        hold on
        
        for trial = 1:1:trials
            time_a_c(trial,g,1) = 0;
            
            time = time_a_c(trial,g,:);
            y = (1/1000)*bat_t(trial,g,:)/3600;
            %y = battery_charge(t,g,:);
            
            y = displacement_c(trial,g,:);
            y = s_v_i(:);
            time = d_i(:);
            y = e_v_f(:);
            
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

close all
figure
y_plots = 2;
x_plots = y_plots;
count = 1;

ax = subplot(x_plots,y_plots,count);
title(ax,'displacement');
hold on
count = count +1;
time = time_a_c(1,1,:);
y = displacement_c(1,1,:);
plot(time(:), y(:))

ax = subplot(x_plots,y_plots,count);
title(ax,'velocity');
hold on
count = count +1;
time = time_a_c(1,1,:);
y = velocity_c(1,1,:);
plot(time(:), y(:))

ax = subplot(x_plots,y_plots,count);
title(ax,'acceleration');
hold on
count = count +1;
time = time_a_c(1,1,:);
y = a_c(:);
plot(time(:), y(:))

ax = subplot(x_plots,y_plots,count);
title(ax,'F_trac');
hold on
count = count +1;
time = time_a_c(1,1,:);

y = force_trac_c(1,1,:);
plot(time(:), y(:))

%% Function Definitions: File Manipulation

function [] = setFILE()
global FILE
FILE = csvread("D:\Google Drive\Part B\Working Documents\Code\Matlab-Repo\Matlab\nedc.csv",1,0);
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

function [duration] = getDuration(duration_interpolated, inc)
import constants.*;
duration = duration_interpolated(inc);
end

function [acceleration] = getAcceleration(acceleration_interpolated, inc)
import constants.*;
acceleration = acceleration_interpolated(inc);
if (isnan(acceleration))
    acceleration = 0;
end
end


function [v] = getVelocity(velocity_interpolated, inc)
import constants.*;
v = velocity_interpolated(inc);
if (isnan(v))
    v = 0;
end
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
d = sqrt(delta_dist^2 + delta_elev^2);
W = F*delta_dist;
end

function [TL_motor] = gear_transform_TL(gear, TL_wheel)
import constants.*;
TL_motor = TL_wheel/gear;
end