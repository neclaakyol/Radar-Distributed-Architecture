function [points, buffer, counters] = radar_parse_bytes(buffer, newBytes, counters)
%RADAR_PARSE_BYTES Streaming parser for encrypted 12-byte radar frames.

if nargin < 3
    counters = radar_init_counters();
end

buffer = uint8([buffer(:).', uint8(newBytes(:).')]);
points = struct("angle_deg", {}, "distance_mm", {}, "sequence", {});

while numel(buffer) >= 2
    sync = find(buffer(1:end-1) == uint8(hex2dec("AA")) & ...
                buffer(2:end) == uint8(hex2dec("55")), 1, "first");

    if isempty(sync)
        keepLast = ~isempty(buffer) && buffer(end) == uint8(hex2dec("AA"));
        dropped = numel(buffer) - double(keepLast);
        counters.sync_drops = counters.sync_drops + max(dropped, 0);
        if keepLast
            buffer = buffer(end);
        else
            buffer = uint8([]);
        end
        return;
    end

    if sync > 1
        counters.sync_drops = counters.sync_drops + sync - 1;
        buffer = buffer(sync:end);
    end

    if numel(buffer) < 12
        return;
    end

    frame = buffer(1:12);
    buffer = buffer(13:end);
    [point, ok] = radar_decode_frame(frame);

    if ok
        point.sequence = counters.valid_frames;
        counters.valid_frames = counters.valid_frames + 1;
        if point.distance_mm == 0
            counters.timeout_readings = counters.timeout_readings + 1;
        end
        points(end + 1) = point; %#ok<AGROW>
    else
        counters.crc_drops = counters.crc_drops + 1;
    end
end
end
