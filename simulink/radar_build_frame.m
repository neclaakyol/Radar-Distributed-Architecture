function frame = radar_build_frame(angle_deg, distance_mm)
%RADAR_BUILD_FRAME Sensor-node framing: 12-byte encrypted telemetry frame.
%   Layout matches TaskSecurity() in sketch_may7d.ino:
%     bytes 1..2   : sync 0xAA 0x55
%     bytes 3..10  : XTEA-encrypted payload [angle, dist_hi, dist_lo, 0 x5]
%     bytes 11..12 : CRC-16/CCITT over bytes 1..10, big-endian

payload    = zeros(1, 8);
payload(1) = mod(round(angle_deg), 256);
d          = min(max(round(distance_mm), 0), 65535);
payload(2) = floor(d / 256);
payload(3) = mod(d, 256);

encrypted = radar_xtea(payload, 'enc');

frame = zeros(12, 1);
frame(1)    = 170;             % 0xAA
frame(2)    = 85;              % 0x55
frame(3:10) = encrypted(:);

crc       = radar_crc16(frame(1:10));
frame(11) = floor(crc / 256);
frame(12) = mod(crc, 256);
end
