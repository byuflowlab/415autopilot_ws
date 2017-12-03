clear; close all;

% ------- your data here -------
% latitude/longitude/altitude (m)/time (s)
lat = [];
long = [];
altitude = [];
t = [];

% ------------------------------

% for plotting circles
theta = linspace(0, 2*pi, 200);  

% ground info
alt_ground = 1508;
ft2m = 0.3048;
floor = 100*ft2m;
ceiling = 400*ft2m;

% ---- waypoints ----
home = [40.26778432216462, -111.63506331014253];
wp1 = [40.26702297859316, -111.63508476781465];
wp2 = [40.267641059243935, -111.63587333726502];
wp3 = [40.26678857, -111.63552403];

% tolerance
R = 10.0;  % m

% distance between points (let home be origin)
[e1, n1, ~] = distance(home, wp1);
[e2, n2, ~] = distance(home, wp2);
[e3, n3, ~] = distance(home, wp3);

n = length(lat);
north = zeros(n);
east = zeros(n);
for i = 1:n
    [north(i), east(i), ~] = distance(home, [lat(i), long(i)]);
end

% ------ boundary -------
ellipsemajor1 = [40.268316, -111.635040];
ellipsemajor2 = [40.266145, -111.636008];
ellipseminor = [40.266994, -111.634407];

% rotation angle
alpha = -15*pi/180.0;

% compute center
center = 0.5*(ellipsemajor1 + ellipsemajor2);
[CE, CN, ~] = distance(home, center);

% semimajor radii
[~, ~, Rmajor] = distance(center, ellipsemajor1);
[~, ~, Rminor] = distance(center, ellipseminor);

xellipse = CE + Rminor*cos(theta)*cos(alpha) - Rmajor*sin(theta)*sin(alpha);
yellipse = CN + Rminor*cos(theta)*sin(alpha) + Rmajor*sin(theta)*cos(alpha);

% ----- plot ------
figure; hold on;
plot(0, 0, 'r*', 'MarkerSize', 24);
plot(e1 + R*cos(theta), n1 + R*sin(theta), 'k');
text(e1, n1, '1', 'FontSize', 18);
plot(e2 + R*cos(theta), n2 + R*sin(theta), 'k');
text(e2, n2, '2', 'FontSize', 18);
plot(e3 + R*cos(theta), n3 + R*sin(theta), 'k');
text(e3, n3, '3', 'FontSize', 18);
plot(north, east, 'k');
plot(xellipse, yellipse, 'r--');
axis('equal');
xlabel('distance east from home (m)');
ylabel('distance north from home (m)');

figure; hold on;
plot(t, altitude-alt_ground);
plot([t(1), t(end)], [floor, floor], 'r--');
plot([t(1), t(end)], [ceiling, ceiling], 'r--');
xlabel('time (s)');
ylabel('height above ground (m)');


function [distE, distN, totaldist] = distance(pt1, pt2)
%  distance of 2 from 1,   pt = [lat, long]

    EARTH_RADIUS = 6371000.0;
    distN = EARTH_RADIUS*(pt2(1) - pt1(1))*pi/180.0;
    distE = EARTH_RADIUS*cos(pt1(1)*pi/180.0)*(pt2(2) - pt1(2))*pi/180.0;
    totaldist = norm([distN, distE]);
end