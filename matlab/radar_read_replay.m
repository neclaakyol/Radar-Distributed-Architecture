function [points, counters] = radar_read_replay(replayPath)
%RADAR_READ_REPLAY Decode a raw replay file into a table of radar points.

replayPath = string(replayPath);
fid = fopen(char(replayPath), "r");
if fid < 0
    error("failed to open replay file: %s", string(replayPath));
end
cleanup = onCleanup(@() fclose(fid));

bytes = fread(fid, Inf, "uint8=>uint8").';
counters = radar_init_counters();
[pointStructs, leftover, counters] = radar_parse_bytes(uint8([]), bytes, counters);

if ~isempty(leftover)
    fprintf("Warning: %d trailing byte(s) did not form a complete frame.\n", numel(leftover));
end

points = radar_points_to_table(pointStructs);
fprintf("valid=%d crc_drops=%d sync_drops=%d timeouts=%d\n", ...
    counters.valid_frames, counters.crc_drops, counters.sync_drops, counters.timeout_readings);
end
