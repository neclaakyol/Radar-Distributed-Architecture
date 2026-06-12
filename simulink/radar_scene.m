function distance_mm = radar_scene(angle_deg, t)
%RADAR_SCENE Simulated tabletop arena seen by the HC-SR04.
%   Returns the echo distance in mm for the given servo angle, or 0 for an
%   echo timeout (same convention as the firmware and C++ backend).
%
%   Two foam-block targets, as in the report's Metric 2 test plan:
%     - bearing 60 deg, starting at 400 mm, drifting radially at 8 mm/s
%       (about 38 mm displacement between forward and return sweep, below
%        the 80 mm association threshold)
%     - bearing 120 deg, static at 600 mm (MTI should report ~zero motion)

targets = [ 60, 400 + 8 * t;
           120, 600 ];

half_beam_deg = 3;
max_range_mm  = 800;

best = inf;
for k = 1:size(targets, 1)
    if abs(angle_deg - targets(k, 1)) <= half_beam_deg
        best = min(best, targets(k, 2));
    end
end

if isfinite(best) && best <= max_range_mm
    distance_mm = round(best);
else
    distance_mm = 0;   % timeout marker
end
end
