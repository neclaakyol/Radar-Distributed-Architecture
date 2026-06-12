function [valid, angle, dist, validCnt, crcDrops] = radar_parse_frame(frame)
%RADAR_PARSE_FRAME Edge-node frame validation and decryption.
%   Mirrors the C++ backend FrameParser: sync check, CRC-16 verification,
%   XTEA decrypt, plus the protocol counters used for Metric 1.

global RADAR_SIM

f     = double(frame(:)');
valid = 0;
angle = 0;
dist  = 0;

expected = radar_crc16(f(1:10));
received = f(11) * 256 + f(12);

if f(1) == 170 && f(2) == 85 && received == expected
    payload = radar_xtea(f(3:10), 'dec');
    angle   = payload(1);
    dist    = payload(2) * 256 + payload(3);
    valid   = 1;
    RADAR_SIM.validCnt = RADAR_SIM.validCnt + 1;
    if dist == 0
        RADAR_SIM.timeouts = RADAR_SIM.timeouts + 1;
    end
else
    RADAR_SIM.crcDrops = RADAR_SIM.crcDrops + 1;
end

validCnt = RADAR_SIM.validCnt;
crcDrops = RADAR_SIM.crcDrops;
end
