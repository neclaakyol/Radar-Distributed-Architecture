function block = radar_xtea_encrypt_block(block)
%RADAR_XTEA_ENCRYPT_BLOCK Encrypt a two-word XTEA block with wraparound math.

block = uint32(block);
key = uint32([hex2dec("12345678"), hex2dec("9ABCDEF0"), ...
              hex2dec("11223344"), hex2dec("55667788")]);
delta = uint32(hex2dec("9E3779B9"));
sumValue = uint32(0);
v0 = block(1);
v1 = block(2);

for round = 1:32 %#ok<FXUP>
    keyIndex = double(bitand(sumValue, uint32(3))) + 1;
    term = bitxor(add32(bitxor(bitshift(v1, 4), bitshift(v1, -5)), v1), ...
                  add32(sumValue, key(keyIndex)));
    v0 = add32(v0, term);
    sumValue = add32(sumValue, delta);

    keyIndex = double(bitand(bitshift(sumValue, -11), uint32(3))) + 1;
    term = bitxor(add32(bitxor(bitshift(v0, 4), bitshift(v0, -5)), v0), ...
                  add32(sumValue, key(keyIndex)));
    v1 = add32(v1, term);
end

block = [v0, v1];
end

function out = add32(a, b)
out = uint32(mod(double(a) + double(b), 2^32));
end
