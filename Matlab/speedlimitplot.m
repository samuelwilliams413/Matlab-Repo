FILE = csvread("D:\Users\Samuel\Downloads\SEQ_roads_lat_long_dis2.csv",1,0);


z = FILE(:,1);
x = FILE(:,2);
y = FILE(:,3);

close all
figure


scatter3(x,y,z)
colormap(jet) 

