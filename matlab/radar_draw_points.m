function radar_draw_points(points, counters, titleText)
%RADAR_DRAW_POINTS Draw a half-PPI style MATLAB plot.

if nargin < 3
    titleText = "Radar Points";
end

valid = points.distance_mm > 0;
theta = deg2rad(points.angle_deg(valid));
range = points.distance_mm(valid);
x = range .* cos(theta);
y = range .* sin(theta);

if isempty(range)
    maxRange = 1000;
else
    maxRange = max(1000, ceil(max(range) / 250) * 250);
end

cla;
hold on;
axis equal;
xlim([-maxRange, maxRange]);
ylim([0, maxRange]);
grid on;
xlabel("x mm");
ylabel("y mm");
title(titleText);

angles = linspace(0, pi, 181);
for ring = linspace(maxRange / 4, maxRange, 4)
    plot(ring * cos(angles), ring * sin(angles), "Color", [0.1 0.45 0.1]);
end
for angleDeg = 0:30:180
    a = deg2rad(angleDeg);
    plot([0, maxRange * cos(a)], [0, maxRange * sin(a)], "Color", [0.08 0.3 0.08]);
end

if ~isempty(x)
    scatter(x, y, 22, range, "filled");
    colormap("summer");
end

status = sprintf("valid=%d  crc=%d  sync=%d  timeouts=%d", ...
    counters.valid_frames, counters.crc_drops, counters.sync_drops, counters.timeout_readings);
text(-maxRange * 0.98, maxRange * 0.95, status, "Color", [0 0.5 0], ...
    "FontName", "Consolas", "FontSize", 10);
hold off;
end
