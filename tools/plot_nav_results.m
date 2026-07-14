% plot_nav_results.m
% =========================================================================
% Đọc file CSV từ nav_data_collector và vẽ đồ thị kết quả thực nghiệm AMR
%
% Cách dùng:
%   1. Mở MATLAB
%   2. cd vào thư mục chứa file .m này
%   3. Chỉnh biến CSV_FILE bên dưới cho đúng đường dẫn
%   4. Run: plot_nav_results
% =========================================================================

clear; clc; close all;

% ── Đường dẫn file CSV ────────────────────────────────────────────────────────
CSV_FILE = '~/amr_nav_data.csv';   % <── Sửa đường dẫn ở đây

% Mở rộng ~ nếu trên Linux/Mac
if startsWith(CSV_FILE, '~')
    CSV_FILE = [getenv('HOME'), CSV_FILE(2:end)];
end

% ── Đọc dữ liệu ───────────────────────────────────────────────────────────────
fprintf('Đọc file: %s\n', CSV_FILE);
T = readtable(CSV_FILE, 'Delimiter', ',', 'VariableNamingRule', 'preserve');

% Lọc theo kết quả
ok_idx   = T.succeeded == 1;
fail_idx = T.succeeded == 0;

trial    = T.trial;
ep_cm    = T.error_pos_cm;
ey_deg   = T.error_yaw_deg;
dur      = T.duration_s;
gx = T.goal_x;  gy = T.goal_y;
ax = T.actual_x; ay = T.actual_y;

n_total   = height(T);
n_ok      = sum(ok_idx);
n_fail    = sum(fail_idx);

fprintf('Tổng: %d lần  |  Thành công: %d  |  Thất bại: %d\n', ...
        n_total, n_ok, n_fail);

% ── Màu sắc ───────────────────────────────────────────────────────────────────
C_blue   = [0.18 0.45 0.75];
C_orange = [0.93 0.54 0.13];
C_green  = [0.20 0.65 0.30];
C_red    = [0.85 0.20 0.20];
C_gray   = [0.70 0.70 0.70];

% ==========================================================================
%  FIGURE 1 — Sai số vị trí theo từng lần thử
% ==========================================================================
figure('Name', 'Sai so vi tri', 'NumberTitle', 'off', ...
       'Position', [50 500 700 320]);

bar_colors = repmat(C_blue, n_total, 1);
bar_colors(fail_idx, :) = repmat(C_red, n_fail, 1);

b = bar(trial, ep_cm, 'FaceColor', 'flat');
b.CData = bar_colors;
hold on;

% Đường trung bình (chỉ tính SUCCEEDED)
if n_ok > 0
    mean_ep = mean(ep_cm(ok_idx));
    yline(mean_ep, '--', sprintf('TB = %.1f cm', mean_ep), ...
          'Color', C_orange, 'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
end

xlabel('Số thứ tự lần thử (trial)');
ylabel('Sai số vị trí (cm)');
title('Sai số vị trí theo từng lần thử');
legend({'Thành công', 'Thất bại'}, 'Location', 'northeast');
% Vẽ legend màu thủ công
bar(nan, nan, 'FaceColor', C_blue);
bar(nan, nan, 'FaceColor', C_red);
legend({'Thành công', 'Thất bại'}, 'Location', 'northeast');

xticks(trial);
grid on;
box on;
set(gca, 'FontSize', 11);

% ==========================================================================
%  FIGURE 2 — Sai số góc theo từng lần thử
% ==========================================================================
figure('Name', 'Sai so goc', 'NumberTitle', 'off', ...
       'Position', [50 150 700 320]);

b2 = bar(trial, ey_deg, 'FaceColor', 'flat');
b2.CData = bar_colors;
hold on;

if n_ok > 0
    mean_ey = mean(ey_deg(ok_idx));
    yline(mean_ey, '--', sprintf('TB = %.2f°', mean_ey), ...
          'Color', C_orange, 'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
end

xlabel('Số thứ tự lần thử (trial)');
ylabel('Sai số góc (°)');
title('Sai số góc quay theo từng lần thử');
xticks(trial);
grid on;
box on;
set(gca, 'FontSize', 11);

% ==========================================================================
%  FIGURE 3 — Bản đồ 2D: Goal vs Thực tế
% ==========================================================================
figure('Name', 'Ban do 2D', 'NumberTitle', 'off', ...
       'Position', [800 350 600 520]);

% Vẽ đường nối goal → actual cho từng lần thử
for i = 1:n_total
    if ok_idx(i)
        c_line = [0.7 0.85 0.95];
    else
        c_line = [1.0 0.80 0.80];
    end
    line([gx(i), ax(i)], [gy(i), ay(i)], ...
         'Color', c_line, 'LineWidth', 1.2);
    hold on;
end

% Vẽ điểm Goal (hình vuông)
scatter(gx(ok_idx),   gy(ok_idx),   90, C_blue,   's', 'filled', ...
        'DisplayName', 'Goal (OK)',       'LineWidth', 0.5);
scatter(gx(fail_idx), gy(fail_idx), 90, C_red,    's', 'filled', ...
        'DisplayName', 'Goal (FAIL)',     'LineWidth', 0.5);

% Vẽ điểm Thực tế (hình tròn)
scatter(ax(ok_idx),   ay(ok_idx),   70, C_green,  'o', 'filled', ...
        'DisplayName', 'Thực tế (OK)',    'LineWidth', 0.5);
scatter(ax(fail_idx), ay(fail_idx), 70, C_orange, 'o', 'filled', ...
        'DisplayName', 'Thực tế (FAIL)', 'LineWidth', 0.5);

% Đánh số lần thử
for i = 1:n_total
    text(gx(i)+0.03, gy(i)+0.05, num2str(trial(i)), ...
         'FontSize', 8, 'Color', [0.3 0.3 0.3]);
end

xlabel('X (m)');
ylabel('Y (m)');
title('Bản đồ 2D: Goal (■) vs Thực tế (●)');
legend('Location', 'best');
axis equal;
grid on;
box on;
set(gca, 'FontSize', 11);

% ==========================================================================
%  FIGURE 4 — Thời gian hoàn thành mỗi lần thử
% ==========================================================================
figure('Name', 'Thoi gian', 'NumberTitle', 'off', ...
       'Position', [800 50 700 280]);

b4 = bar(trial, dur, 'FaceColor', 'flat');
b4.CData = bar_colors;
hold on;

if n_ok > 0
    mean_dur = mean(dur(ok_idx));
    yline(mean_dur, '--', sprintf('TB = %.1f s', mean_dur), ...
          'Color', C_orange, 'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
end

xlabel('Số thứ tự lần thử (trial)');
ylabel('Thời gian (s)');
title('Thời gian hoàn thành từng lần thử');
xticks(trial);
grid on;
box on;
set(gca, 'FontSize', 11);

% ==========================================================================
%  FIGURE 5 — Phân phối sai số (histogram)
% ==========================================================================
if n_ok >= 3
    figure('Name', 'Phan phoi sai so', 'NumberTitle', 'off', ...
           'Position', [50 50 680 360]);

    subplot(1, 2, 1);
    histogram(ep_cm(ok_idx), 'FaceColor', C_blue, 'EdgeColor', 'w', ...
              'Normalization', 'count');
    xlabel('Sai số vị trí (cm)');
    ylabel('Số lần');
    title('Phân phối sai số vị trí');
    grid on;
    set(gca, 'FontSize', 11);

    subplot(1, 2, 2);
    histogram(ey_deg(ok_idx), 'FaceColor', C_green, 'EdgeColor', 'w', ...
              'Normalization', 'count');
    xlabel('Sai số góc (°)');
    ylabel('Số lần');
    title('Phân phối sai số góc');
    grid on;
    set(gca, 'FontSize', 11);

    sgtitle('Phân phối sai số (chỉ SUCCEEDED)', 'FontSize', 13);
end

% ==========================================================================
%  In bảng thống kê tổng kết ra Command Window
% ==========================================================================
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('  THỐNG KÊ TỔNG KẾT (chỉ SUCCEEDED, n=%d)\n', n_ok);
fprintf('%s\n', repmat('=', 1, 60));

if n_ok > 0
    fprintf('  Sai số VỊ TRÍ:\n');
    fprintf('    Trung bình : %.2f cm\n', mean(ep_cm(ok_idx)));
    fprintf('    Độ lệch chuẩn : %.2f cm\n', std(ep_cm(ok_idx)));
    fprintf('    Min / Max  : %.2f / %.2f cm\n', ...
            min(ep_cm(ok_idx)), max(ep_cm(ok_idx)));

    fprintf('  Sai số GÓC:\n');
    fprintf('    Trung bình : %.2f °\n', mean(ey_deg(ok_idx)));
    fprintf('    Độ lệch chuẩn : %.2f °\n', std(ey_deg(ok_idx)));
    fprintf('    Min / Max  : %.2f / %.2f °\n', ...
            min(ey_deg(ok_idx)), max(ey_deg(ok_idx)));

    fprintf('  THỜI GIAN:\n');
    fprintf('    Trung bình : %.1f s\n', mean(dur(ok_idx)));
    fprintf('    Min / Max  : %.1f / %.1f s\n', ...
            min(dur(ok_idx)), max(dur(ok_idx)));
end
fprintf('%s\n\n', repmat('=', 1, 60));
