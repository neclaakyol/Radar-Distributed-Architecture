function radar_ppi_draw(angle, valid, dist, cycleDone) %#ok<INUSD>
%RADAR_PPI_DRAW Live half-PPI preview figure (stands in for the Vulkan UI).
%   Phosphor-green semicircular display: range rings, rotating sweep line,
%   current-sweep blips, previous-sweep history, and MTI vectors as arrows.

global RADAR_SIM
if RADAR_SIM.headless
    return;
end

persistent fig ax sweepLine curScat histScat vecLines callCount
if isempty(callCount)
    callCount = 0;
end
callCount = callCount + 1;
if mod(callCount, 4) ~= 0 && cycleDone < 0.5
    return;   % throttle redraw rate
end

R = RADAR_SIM.max_range_mm;

if isempty(fig) || ~isgraphics(fig)
    fig = figure('Name', 'Radar PPI - Edge Node Preview', ...
                 'Color', 'k', 'NumberTitle', 'off');
    ax = axes(fig, 'Color', 'k', 'XColor', [0 0.5 0.2], 'YColor', [0 0.5 0.2]);
    hold(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [-R - 50, R + 50]);
    ylim(ax, [0, R + 50]);
    th = linspace(0, 180, 181);
    for r = [R / 3, 2 * R / 3, R]
        plot(ax, r * cosd(th), r * sind(th), '-', 'Color', [0 0.35 0.15]);
    end
    for a = 0:30:180
        plot(ax, [0, R * cosd(a)], [0, R * sind(a)], '-', 'Color', [0 0.25 0.1]);
    end
    sweepLine = plot(ax, [0 0], [0 0], '-', 'Color', [0.4 1 0.5], 'LineWidth', 1.5);
    curScat   = scatter(ax, nan, nan, 36, [0.5 1 0.6], 'filled');
    histScat  = scatter(ax, nan, nan, 16, [0.1 0.6 0.3], 'filled');
    vecLines  = gobjects(0);
    title(ax, sprintf('Half-PPI  |  blips + MTI vectors (x%d)', ...
          RADAR_SIM.vector_draw_scale), 'Color', [0.4 1 0.5]);
end

set(sweepLine, 'XData', [0, R * cosd(angle)], 'YData', [0, R * sind(angle)]);

buf = RADAR_SIM.buf;
if isempty(buf)
    set(curScat, 'XData', nan, 'YData', nan);
else
    set(curScat, 'XData', buf(:, 2) .* cosd(buf(:, 1)), ...
                 'YData', buf(:, 2) .* sind(buf(:, 1)));
end

hist = [RADAR_SIM.lastFwd; RADAR_SIM.lastRet];
if ~isempty(hist)
    set(histScat, 'XData', hist(:, 1), 'YData', hist(:, 2));
end

delete(vecLines(isgraphics(vecLines)));
vecLines = gobjects(0);
V = RADAR_SIM.vectors;
s = RADAR_SIM.vector_draw_scale;
for i = 1:size(V, 1)
    dx = (V(i, 3) - V(i, 1)) * s;
    dy = (V(i, 4) - V(i, 2)) * s;
    vecLines(end + 1) = quiver(ax, V(i, 1), V(i, 2), dx, dy, 0, ...
        'Color', [1 0.85 0.3], 'LineWidth', 1.4, 'MaxHeadSize', 0.6); %#ok<AGROW>
end

drawnow limitrate
end
