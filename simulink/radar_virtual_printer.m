function nbytes = radar_virtual_printer(bytes)
%RADAR_VIRTUAL_PRINTER Software 58 mm thermal printer.
%   Decodes the incoming ESC/POS byte stream (it has no knowledge of how
%   the stream was produced), renders the receipt, and saves it as a PNG in
%   simulink/prints/. This proves the encoder output is hardware-correct
%   before the physical printer arrives.
%
%   Supported commands: ESC @, ESC a n, ESC d n, GS v 0 (raster), GS V m.

global RADAR_SIM

b      = double(bytes(:)');
nbytes = numel(b);

ops     = {};
curline = '';
i = 1;
while i <= nbytes
    c = b(i);
    if c == 27 && i + 1 <= nbytes                 % ESC
        switch b(i + 1)
            case 100                              % ESC d n : feed n lines
                ops{end + 1} = {'feed', b(i + 2)}; %#ok<AGROW>
                i = i + 3;
            case 97                               % ESC a n : justification
                i = i + 3;
            otherwise                             % ESC @ and others
                i = i + 2;
        end
    elseif c == 29 && i + 1 <= nbytes             % GS
        if b(i + 1) == 118                        % GS v 0 : raster image
            xb = b(i + 4) + 256 * b(i + 5);
            yy = b(i + 6) + 256 * b(i + 7);
            n  = xb * yy;
            data = b(i + 8 : i + 7 + n);
            img  = false(yy, xb * 8);
            for row = 1:yy
                rowBytes = data((row - 1) * xb + (1:xb));
                bits = rem(floor(rowBytes(:) * 2 .^ (-7:0)), 2);  % xb x 8, col 1 = bit7
                img(row, :) = reshape(bits', 1, []) > 0;
            end
            ops{end + 1} = {'raster', img}; %#ok<AGROW>
            i = i + 8 + n;
        elseif b(i + 1) == 86                     % GS V m : cut
            i = i + 3;
        else
            i = i + 2;
        end
    elseif c == 10                                % LF: flush text line
        ops{end + 1} = {'text', curline}; %#ok<AGROW>
        curline = '';
        i = i + 1;
    else
        curline(end + 1) = char(c); %#ok<AGROW>
        i = i + 1;
    end
end

render_receipt(ops);
fprintf('[printer] cycle %03d: received %d bytes\n', RADAR_SIM.cycleCount, nbytes);
end

function render_receipt(ops)
global RADAR_SIM

lineH = 16;
total = 24;
for k = 1:numel(ops)
    switch ops{k}{1}
        case 'text',   total = total + lineH;
        case 'raster', total = total + size(ops{k}{2}, 1) + 8;
        case 'feed',   total = total + 8 * ops{k}{2};
    end
end

if RADAR_SIM.headless
    vis = 'off';
else
    vis = 'on';
end

fig = figure('Visible', vis, 'Color', [0.92 0.92 0.92], ...
             'Position', [80 80 460 min(total + 80, 900)], ...
             'Name', sprintf('Virtual Thermal Printer - cycle %03d', RADAR_SIM.cycleCount), ...
             'NumberTitle', 'off');
ax = axes(fig, 'Position', [0.05 0.02 0.90 0.94]);
hold(ax, 'on');
set(ax, 'YDir', 'reverse', 'XTick', [], 'YTick', []);
xlim(ax, [0 404]);
ylim(ax, [0 total]);
daspect(ax, [1 1 1]);

% paper
fill(ax, [10 394 394 10], [0 0 total total], 'w', 'EdgeColor', [0.7 0.7 0.7]);

y = 18;
for k = 1:numel(ops)
    switch ops{k}{1}
        case 'text'
            text(ax, 202, y, ops{k}{2}, 'FontName', 'Courier New', ...
                 'FontSize', 9, 'HorizontalAlignment', 'center', ...
                 'Interpreter', 'none');
            y = y + lineH;
        case 'raster'
            img = ops{k}{2};
            [h, w] = size(img);
            cdata = repmat(uint8(~img) * 255, 1, 1, 3);
            image(ax, 'XData', [202 - w / 2, 202 + w / 2], ...
                      'YData', [y, y + h], 'CData', cdata);
            y = y + h + 8;
        case 'feed'
            y = y + 8 * ops{k}{2};
    end
end

png = fullfile(RADAR_SIM.printDir, ...
               sprintf('receipt_cycle_%03d.png', RADAR_SIM.cycleCount));
try
    exportgraphics(fig, png);
catch
    % headless fallback: dump the raw raster only
    for k = 1:numel(ops)
        if strcmp(ops{k}{1}, 'raster')
            imwrite(uint8(~ops{k}{2}) * 255, png);
            break;
        end
    end
end

if RADAR_SIM.headless
    close(fig);
end
end
