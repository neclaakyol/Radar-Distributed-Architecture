function value = radar_load_u32_le(bytes)
%RADAR_LOAD_U32_LE Load four little-endian bytes as uint32.

bytes = uint8(bytes(:).');
if numel(bytes) ~= 4
    error("radar_load_u32_le requires exactly four bytes");
end

value = bitor( ...
    bitor(uint32(bytes(1)), bitshift(uint32(bytes(2)), 8)), ...
    bitor(bitshift(uint32(bytes(3)), 16), bitshift(uint32(bytes(4)), 24)));
end
