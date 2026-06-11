function [point, ok, info] = radar_decode_frame(frame)
%RADAR_DECODE_FRAME Validate CRC and decrypt one 12-byte telemetry frame.

frame = uint8(frame(:).');
point = struct("angle_deg", 0, "distance_mm", 0, "sequence", 0);
ok = false;
info = struct("calculated_crc", uint16(0), "received_crc", uint16(0), "reason", "");

if numel(frame) ~= 12
    info.reason = "wrong frame length";
    return;
end

if frame(1) ~= uint8(hex2dec("AA")) || frame(2) ~= uint8(hex2dec("55"))
    info.reason = "bad sync";
    return;
end

info.calculated_crc = radar_crc16_ccitt(frame(1:10));
info.received_crc = bitor(bitshift(uint16(frame(11)), 8), uint16(frame(12)));
if info.calculated_crc ~= info.received_crc
    info.reason = "crc mismatch";
    return;
end

encrypted = frame(3:10);
block = [radar_load_u32_le(encrypted(1:4)), radar_load_u32_le(encrypted(5:8))];
plainBlock = radar_xtea_decrypt_block(block);
plain = [radar_store_u32_le(plainBlock(1)), radar_store_u32_le(plainBlock(2))];

point.angle_deg = double(plain(1));
point.distance_mm = double(bitor(bitshift(uint16(plain(2)), 8), uint16(plain(3))));
ok = true;
end
