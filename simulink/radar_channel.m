function out = radar_channel(frame, noise_on)
%RADAR_CHANNEL UART link model with optional EMI bit-flip noise.
%   With noise enabled, every bit flips independently with p = 2e-4, which
%   corrupts roughly 2% of 96-bit frames -- enough to make the CRC drop
%   counter move during the Metric 1 demo.

out = double(frame(:));
if noise_on > 0.5
    p = 2e-4;
    for i = 1:numel(out)
        for bit = 0:7
            if rand < p
                out(i) = bitxor(out(i), 2^bit);
            end
        end
    end
end
end
