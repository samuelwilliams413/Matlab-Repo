min_and_max = @(x) [min(x), max(x)];
min_and_max([3 4 1 6 2])


% Transform km/hr to m/s
kmhr_to_ms = @ (velocity_kmhr) [velocity_kmhr/3.6];

kmhr_to_ms(10)