function bytes = radar_escpos_encode()
%RADAR_ESCPOS_ENCODE Stage 5: PPI snapshot -> real ESC/POS byte stream.
%   Produces the exact command sequence a 58 mm thermal printer expects:
%     ESC @            initialize
%     ESC a 1          center justification
%     <text> LF        header lines
%     GS v 0           raster bit image, 384 dots wide (48 bytes/row)
%     ESC d n          paper feed
%     GS V 0           full cut
%   The same encoder logic ports directly to the C++ backend later.

global RADAR_SIM

img = render_ppi_raster(RADAR_SIM);
[H, W] = size(img);
xbytes = W / 8;

line1 = 'RADAR TACTICAL LOG';
line2 = sprintf('CYCLE %03d  MTI VECTORS %d', ...
                RADAR_SIM.cycleCount, size(RADAR_SIM.vectors, 1));
line3 = sprintf('VALID %d  CRC DROPS %d', ...
                RADAR_SIM.validCnt, RADAR_SIM.crcDrops);
line4 = sprintf('CENG424  TAU %dMM  RANGE %dMM', ...
                RADAR_SIM.tau_mm, RADAR_SIM.max_range_mm);

bytes = [27 64, ...                                  % ESC @
         27 97 1, ...                                % ESC a 1
         double(line1) 10, ...
         double(line2) 10, ...
         double(line3) 10, ...
         29 118 48 0, ...                            % GS v 0, m = 0
         mod(xbytes, 256) floor(xbytes / 256), ...
         mod(H, 256) floor(H / 256)];

weights = 2 .^ (7:-1:0);                             % bit 7 = leftmost dot
raster  = zeros(1, H * xbytes);
for row = 1:H
    raster((row - 1) * xbytes + (1:xbytes)) = ...
        weights * double(reshape(img(row, :), 8, xbytes));
end

bytes = [bytes, raster, ...
         double(line4) 10, ...
         27 100 3, ...                               % ESC d 3 (feed)
         29 86 0];                                   % GS V 0 (cut)
end

function img = render_ppi_raster(S)
% Rasterize the PPI state into a 384-dot-wide 1-bit image (true = black).
W = 384;
H = 216;
img = false(H, W);

cx    = 192.5;
cy    = 206;
scale = 180 / S.max_range_mm;

% Range rings and bearing spokes
for r_mm = [S.max_range_mm / 3, 2 * S.max_range_mm / 3, S.max_range_mm]
    for th = 0:0.5:180
        img = set_px(img, cx + r_mm * scale * cosd(th), ...
                          cy - r_mm * scale * sind(th));
    end
end
for a = 0:30:180
    img = draw_seg(img, cx, cy, ...
                   cx + S.max_range_mm * scale * cosd(a), ...
                   cy - S.max_range_mm * scale * sind(a));
end

% Blips: forward sweep as large discs, return sweep as small discs
img = draw_blips(img, S.lastFwd, cx, cy, scale, 3);
img = draw_blips(img, S.lastRet, cx, cy, scale, 2);

% MTI vectors as arrows (scaled for legibility)
k = S.vector_draw_scale;
for i = 1:size(S.vectors, 1)
    x1 = cx + S.vectors(i, 1) * scale;
    y1 = cy - S.vectors(i, 2) * scale;
    x2 = cx + (S.vectors(i, 1) + (S.vectors(i, 3) - S.vectors(i, 1)) * k) * scale;
    y2 = cy - (S.vectors(i, 2) + (S.vectors(i, 4) - S.vectors(i, 2)) * k) * scale;
    img = draw_seg(img, x1, y1, x2, y2);
    % arrowhead
    u = [x2 - x1, y2 - y1];
    n = norm(u);
    if n > 1
        u = u / n;
        for rot = [150, -150]
            v = [u(1) * cosd(rot) - u(2) * sind(rot), ...
                 u(1) * sind(rot) + u(2) * cosd(rot)] * 5;
            img = draw_seg(img, x2, y2, x2 + v(1), y2 + v(2));
        end
    end
end
end

function img = draw_blips(img, pts, cx, cy, scale, radius)
for i = 1:size(pts, 1)
    px = cx + pts(i, 1) * scale;
    py = cy - pts(i, 2) * scale;
    for dx = -radius:radius
        for dy = -radius:radius
            if dx^2 + dy^2 <= radius^2
                img = set_px(img, px + dx, py + dy);
            end
        end
    end
end
end

function img = draw_seg(img, x1, y1, x2, y2)
n = max(abs(x2 - x1), abs(y2 - y1));
for i = 0:max(round(n), 1)
    f = i / max(round(n), 1);
    img = set_px(img, x1 + (x2 - x1) * f, y1 + (y2 - y1) * f);
end
end

function img = set_px(img, x, y)
x = round(x);
y = round(y);
if x >= 1 && y >= 1 && x <= size(img, 2) && y <= size(img, 1)
    img(y, x) = true;
end
end
