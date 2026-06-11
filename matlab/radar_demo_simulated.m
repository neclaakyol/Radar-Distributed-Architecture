function points = radar_demo_simulated()
%RADAR_DEMO_SIMULATED Generate, decode, and plot synthetic radar frames.

outputPath = fullfile("matlab", "generated_frames.bin");
radar_generate_replay(outputPath, 2);
points = radar_read_replay(outputPath);
fprintf("Decoded %d synthetic radar points from %s\n", height(points), outputPath);
radar_plot_replay(outputPath);
end
