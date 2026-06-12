function out = radar_xtea(in8, mode)
%RADAR_XTEA Encrypt/decrypt one 64-bit XTEA block (32 rounds).
%   in8  : 8 bytes (values 0..255), little-endian uint32 words, matching the
%          AVR byte layout in sketch_may7d.ino ((uint8_t*)payloadBlock).
%   mode : 'enc' or 'dec'
%   Same key as the firmware and the C++ backend.

key   = [hex2dec('12345678'), hex2dec('9ABCDEF0'), ...
         hex2dec('11223344'), hex2dec('55667788')];
M     = 2^32;
delta = hex2dec('9E3779B9');

b  = double(in8(:)');
v0 = b(1) + b(2)*2^8 + b(3)*2^16 + b(4)*2^24;
v1 = b(5) + b(6)*2^8 + b(7)*2^16 + b(8)*2^24;

if strcmp(mode, 'enc')
    s = 0;
    for i = 1:32
        v0 = mod(v0 + bitxor(feistel(v1), mod(s + key(bitand(s, 3) + 1), M)), M);
        s  = mod(s + delta, M);
        v1 = mod(v1 + bitxor(feistel(v0), mod(s + key(bitand(bitshift(s, -11), 3) + 1), M)), M);
    end
else
    s = mod(delta * 32, M);
    for i = 1:32
        v1 = mod(v1 - bitxor(feistel(v0), mod(s + key(bitand(bitshift(s, -11), 3) + 1), M)), M);
        s  = mod(s - delta, M);
        v0 = mod(v0 - bitxor(feistel(v1), mod(s + key(bitand(s, 3) + 1), M)), M);
    end
end

out = [byte(v0, 0), byte(v0, 1), byte(v0, 2), byte(v0, 3), ...
       byte(v1, 0), byte(v1, 1), byte(v1, 2), byte(v1, 3)];
end

function f = feistel(v)
% (((v << 4) ^ (v >> 5)) + v) with uint32 wraparound
M = 2^32;
f = mod(bitxor(mod(v * 16, M), floor(v / 32)) + v, M);
end

function b = byte(v, k)
b = mod(floor(v / 2^(8 * k)), 256);
end
