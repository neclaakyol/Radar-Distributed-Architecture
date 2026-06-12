function run_demo(noise_on)
%RUN_DEMO Build (if needed) and run the radar model for the advisor demo.
%   run_demo      - clean link: watch the PPI figure, 2 receipts print
%   run_demo(1)   - EMI injection on: CRC Drops counter increments while
%                   valid telemetry keeps flowing (Metric 1 demo)

if nargin < 1
    noise_on = 0;
end

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);

mdl = 'radar_distributed';
if ~exist(fullfile(thisDir, [mdl '.slx']), 'file')
    build_radar_model();
end

load_system(mdl);
set_param([mdl '/EMI Injection'], 'Value', num2str(noise_on));
% Pace simulation to wall-clock so the sweep looks like the real servo.
set_param(mdl, 'EnablePacing', 'on', 'PacingRate', '1');
open_system(mdl);
sim(mdl);
set_param(mdl, 'EnablePacing', 'off');

global RADAR_SIM
fprintf('\n--- Run summary ---\n');
fprintf('Sweeps: %d  Cycles: %d  Valid frames: %d  CRC drops: %d  Timeouts: %d\n', ...
    RADAR_SIM.sweepCount, RADAR_SIM.cycleCount, RADAR_SIM.validCnt, ...
    RADAR_SIM.crcDrops, RADAR_SIM.timeouts);
fprintf('Receipts saved in %s\n', RADAR_SIM.printDir);
end
