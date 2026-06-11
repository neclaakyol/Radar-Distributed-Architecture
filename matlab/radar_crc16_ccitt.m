function crc = radar_crc16_ccitt(bytes)
%RADAR_CRC16_CCITT CRC-16/CCITT-FALSE used by the Arduino and C++ backend.

bytes = uint8(bytes(:).');
crc32 = uint32(hex2dec("FFFF"));
poly = uint32(hex2dec("1021"));

for byte = bytes
    crc32 = bitxor(crc32, bitshift(uint32(byte), 8));
    for bit = 1:8 %#ok<FXUP>
        if bitand(crc32, uint32(hex2dec("8000"))) ~= 0
            crc32 = bitxor(bitshift(crc32, 1), poly);
        else
            crc32 = bitshift(crc32, 1);
        end
        crc32 = bitand(crc32, uint32(hex2dec("FFFF")));
    end
end

crc = uint16(crc32);
end
