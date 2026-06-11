function radar_generate_replay(outputPath, cycles)
%RADAR_GENERATE_REPLAY Write encrypted raw UART frames for offline testing.

if nargin < 2
    cycles = 2;
end

if cycles <= 0
    error("cycles must be positive");
end

outputPath = string(outputPath);
folder = fileparts(outputPath);
if strlength(folder) > 0 && ~isfolder(folder)
    mkdir(folder);
end

fid = fopen(char(outputPath), "w");
if fid < 0
    error("failed to open replay output: %s", outputPath);
end
cleanup = onCleanup(@() fclose(fid));

for cycle = 0:(cycles - 1)
    for angle = 0:180
        distance = uint16(700 + mod(angle, 23) + cycle * 10);
        fwrite(fid, radar_make_frame(uint8(angle), distance), "uint8");
    end
    for angle = 179:-1:0
        distance = uint16(760 + mod(angle, 23) + cycle * 10);
        fwrite(fid, radar_make_frame(uint8(angle), distance), "uint8");
    end
end
end
