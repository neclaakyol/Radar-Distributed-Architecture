function radar_plot_replay(replayPath)
%RADAR_PLOT_REPLAY Plot decoded replay points in a half-PPI view.

[points, counters] = radar_read_replay(replayPath);
figure("Name", "Radar Replay");
radar_draw_points(points, counters, "Replay: " + string(replayPath));
end
