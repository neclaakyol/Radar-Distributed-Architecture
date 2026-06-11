function radar_live_serial(port, baud, durationSeconds)
%RADAR_LIVE_SERIAL Read live encrypted UART frames and plot them.

if nargin < 2 || isempty(baud)
    baud = 115200;
end
if nargin < 3
    durationSeconds = Inf;
end

port = string(port);
serialObj = serialport(port, baud, "Timeout", 0.1);
flush(serialObj);

buffer = uint8([]);
counters = radar_init_counters();
pointStructs = struct("angle_deg", {}, "distance_mm", {}, "sequence", {});

fig = figure("Name", "Live Radar Serial");
startTime = tic;
lastDraw = tic;

while ishandle(fig) && toc(startTime) < durationSeconds
    available = serialObj.NumBytesAvailable;
    if available > 0
        newBytes = read(serialObj, available, "uint8");
        [newPoints, buffer, counters] = radar_parse_bytes(buffer, newBytes, counters);
        if ~isempty(newPoints)
            pointStructs = [pointStructs, newPoints]; %#ok<AGROW>
            if numel(pointStructs) > 720
                pointStructs = pointStructs((end - 719):end);
            end
        end
    end

    if toc(lastDraw) > 0.05
        points = radar_points_to_table(pointStructs);
        radar_draw_points(points, counters, "Live serial: " + port);
        drawnow limitrate;
        lastDraw = tic;
    else
        pause(0.005);
    end
end
end
