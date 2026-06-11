function frame = radar_make_frame(angleDeg, distanceMm)
%RADAR_MAKE_FRAME Create one encrypted 12-byte radar telemetry frame.

payload = zeros(1, 8, "uint8");
payload(1) = uint8(angleDeg);
distanceMm = uint16(distanceMm);
payload(2) = uint8(bitshift(distanceMm, -8));
payload(3) = uint8(bitand(distanceMm, uint16(255)));

block = [radar_load_u32_le(payload(1:4)), radar_load_u32_le(payload(5:8))];
encryptedBlock = radar_xtea_encrypt_block(block);
encrypted = [radar_store_u32_le(encryptedBlock(1)), radar_store_u32_le(encryptedBlock(2))];

frame = zeros(1, 12, "uint8");
frame(1) = uint8(hex2dec("AA"));
frame(2) = uint8(hex2dec("55"));
frame(3:10) = encrypted;

crc = radar_crc16_ccitt(frame(1:10));
frame(11) = uint8(bitshift(crc, -8));
frame(12) = uint8(bitand(crc, uint16(255)));
end
