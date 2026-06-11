function bytes = radar_store_u32_le(value)
%RADAR_STORE_U32_LE Store uint32 as four little-endian bytes.

value = uint32(value);
bytes = zeros(1, 4, "uint8");
bytes(1) = uint8(bitand(value, uint32(255)));
bytes(2) = uint8(bitand(bitshift(value, -8), uint32(255)));
bytes(3) = uint8(bitand(bitshift(value, -16), uint32(255)));
bytes(4) = uint8(bitand(bitshift(value, -24), uint32(255)));
end
