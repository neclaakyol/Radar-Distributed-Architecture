function [sweeps, cycles, nvec, cycleDone] = radar_mti_step(valid, angle, dist, t) %#ok<INUSD>
%RADAR_MTI_STEP Sweep buffering, direction detection, and MTI correlation.
%   Mirrors the backend SweepBuilder + compute_motion_cpu():
%     - buffers valid (angle, dist) points, excluding timeout (dist == 0)
%     - finalizes a sweep on servo direction reversal
%     - clusters contiguous echo runs into one centroid target per object
%     - nearest-neighbor matches the new sweep against the previous
%       opposite-direction sweep within tau_mm
%   A completed return sweep closes a bidirectional cycle (cycleDone = 1),
%   which triggers the thermal-printer stage.

global RADAR_SIM
S = RADAR_SIM;
cycleDone = 0;

if valid > 0.5
    if ~isnan(S.prevAngle) && angle ~= S.prevAngle
        newDir = sign(angle - S.prevAngle);
        if S.dir ~= 0 && newDir ~= S.dir
            % Direction reversed: finalize the sweep that just completed.
            pts  = cluster_targets(S.buf);
            cart = zeros(0, 2);
            if ~isempty(pts)
                cart = [pts(:, 2) .* cosd(pts(:, 1)), pts(:, 2) .* sind(pts(:, 1))];
            end
            S.sweepCount = S.sweepCount + 1;

            if S.dir > 0
                prevOpp   = S.lastRet;
                S.lastFwd = cart;
            else
                prevOpp   = S.lastFwd;
                S.lastRet = cart;
            end

            if ~isempty(prevOpp) && ~isempty(cart)
                S.vectors = nn_match(prevOpp, cart, S.tau_mm);
            end

            if S.dir < 0
                S.cycleCount = S.cycleCount + 1;
                cycleDone = 1;
            end
            S.buf = zeros(0, 2);
        end
        S.dir = newDir;
    end

    if dist > 0
        S.buf(end + 1, :) = [angle, dist];
    end
    S.prevAngle = angle;
end

sweeps = S.sweepCount;
cycles = S.cycleCount;
nvec   = size(S.vectors, 1);
RADAR_SIM = S;
end

function out = cluster_targets(buf)
% Merge contiguous echo runs (adjacent beam positions hitting the same
% object) into a single centroid [angle_deg, dist_mm] per target.
out = zeros(0, 2);
if isempty(buf)
    return;
end
groupStart = 1;
for i = 2:size(buf, 1) + 1
    if i <= size(buf, 1) && ...
       abs(buf(i, 1) - buf(i - 1, 1)) <= 4 && ...
       abs(buf(i, 2) - buf(i - 1, 2)) <= 60
        continue;
    end
    out(end + 1, :) = mean(buf(groupStart:i - 1, :), 1); %#ok<AGROW>
    groupStart = i;
end
end

function vecs = nn_match(prev, cur, tau_mm)
% Greedy nearest-neighbor association with duplicate-assignment resolution,
% as in backend/src/mti_cpu.cpp. Rows: [x_prev y_prev x_cur y_cur] in mm.
vecs = zeros(0, 4);
used = false(size(prev, 1), 1);
for i = 1:size(cur, 1)
    best  = inf;
    bestJ = 0;
    for j = 1:size(prev, 1)
        if used(j)
            continue;
        end
        d = norm(cur(i, :) - prev(j, :));
        if d < best
            best  = d;
            bestJ = j;
        end
    end
    if bestJ > 0 && best <= tau_mm
        used(bestJ) = true;
        vecs(end + 1, :) = [prev(bestJ, :), cur(i, :)]; %#ok<AGROW>
    end
end
end
