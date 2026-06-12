function crc = radar_crc16(bytes)
%RADAR_CRC16 CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), bit-identical
%   to crc16_ccitt() in Arduino/sketch_may7d/sketch_may7d.ino.

crc = 65535;                       % 0xFFFF
for i = 1:numel(bytes)
    crc = bitxor(crc, bitshift(double(bytes(i)), 8));
    for j = 1:8
        if bitand(crc, 32768)      % 0x8000
            crc = bitand(bitxor(bitshift(crc, 1), 4129), 65535);  % ^0x1021
        else
            crc = bitand(bitshift(crc, 1), 65535);
        end
    end
end
end
