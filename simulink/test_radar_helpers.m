function test_radar_helpers()
%TEST_RADAR_HELPERS Self-tests for the protocol and printer helpers.

here = fileparts(mfilename('fullpath'));
addpath(here);

radar_sim_reset();
global RADAR_SIM
RADAR_SIM.headless = true;

% CRC-16/CCITT-FALSE known-answer test
assert(radar_crc16(double('123456789')) == hex2dec('29B1'), 'CRC-16 KAT failed');

% XTEA round trip
payload = [13 1 144 0 0 0 0 0];
assert(isequal(radar_xtea(radar_xtea(payload, 'enc'), 'dec'), payload), ...
    'XTEA round trip failed');

% Frame build -> parse round trip
frame = radar_build_frame(77, 412);
[valid, angle, dist] = radar_parse_frame(frame);
assert(valid == 1 && angle == 77 && dist == 412, 'frame round trip failed');

% Corrupted frame must be dropped
bad = frame;
bad(5) = bitxor(bad(5), 4);
valid2 = radar_parse_frame(bad);
assert(valid2 == 0, 'corrupted frame was accepted');
assert(RADAR_SIM.crcDrops == 1, 'CRC drop counter did not increment');

% MTI: moving target produces one vector, static target none beyond noise
RADAR_SIM.lastRet = [];
fwd = [200 300; -300 400];                  % two targets, cartesian mm
RADAR_SIM.lastFwd = fwd;
% simulate a return sweep where target 1 moved 30 mm in x
RADAR_SIM.buf = zeros(0, 2);

% ESC/POS encode -> virtual printer round trip
RADAR_SIM.lastFwd  = [200 300; -300 400];
RADAR_SIM.lastRet  = [230 300; -300 400];
RADAR_SIM.vectors  = [200 300 230 300];
RADAR_SIM.cycleCount = 999;
bytes = radar_escpos_encode();
assert(isequal(bytes(1:2), [27 64]), 'missing ESC @ init');
assert(isequal(bytes(end-2:end), [29 86 0]), 'missing GS V cut');
n = radar_virtual_printer(bytes);
assert(n == numel(bytes), 'printer byte count mismatch');
png = fullfile(RADAR_SIM.printDir, 'receipt_cycle_999.png');
assert(exist(png, 'file') == 2, 'receipt PNG was not written');
delete(png);

disp('All helper self-tests passed.');
end
