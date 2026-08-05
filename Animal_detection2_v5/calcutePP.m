% 调用函数 心率呼吸率计算
function [RR_bpm, HR_robust_bpm]=calcutePP(I,Q)



%% ── 呼吸率鲁棒计算（CWT 0.1–0.5 Hz 带通 + FFT 找峰）──────────
RR_LOW  = 0.1;   % 呼吸率频段下限 (Hz) → 6 BPM
RR_HIGH = 0.5;   % 呼吸率频段上限 (Hz) → 30 BPM

% CWT 带通：提取呼吸频段 I/Q
[wt_I_rr, f_cwt_rr] = cwt(real(original24G), fs);
[wt_Q_rr,          ] = cwt(imag(original24G), fs);
mask_rr = (f_cwt_rr >= RR_LOW) & (f_cwt_rr <= RR_HIGH);
wt_I_rr(~mask_rr, :) = 0;
wt_Q_rr(~mask_rr, :) = 0;
I_cwt_rr = icwt(wt_I_rr);
Q_cwt_rr = icwt(wt_Q_rr);
cwt_rr   = I_cwt_rr(:) + 1i * Q_cwt_rr(:);

% FFT 找呼吸率峰值
[f_rr, P_rr] = FFT_SHIFT(t_vec, cwt_rr - mean(cwt_rr));
amp_rr   = abs(P_rr);
valid_rr = (f_rr >= RR_LOW) & (f_rr <= RR_HIGH);
f_vrr    = f_rr(valid_rr);
a_vrr    = amp_rr(valid_rr);

[~, locs_rr] = findpeaks(a_vrr, 'SortStr', 'descend', ...
    'MinPeakProminence', 0.1 * (max(a_vrr) - min(a_vrr)));

if isempty(locs_rr)
    warning('呼吸率频段内未检测到峰值。');
    RR_bpm     = NaN;
    RR_freq_hz = NaN;
else
    RR_freq_hz = f_vrr(locs_rr(1));
    RR_bpm     = RR_freq_hz * 60;
    fprintf('=== 呼吸率: %.1f BPM (%.3f Hz) ===\n', RR_bpm, RR_freq_hz);
end

figure;
plot(f_rr, amp_rr, 'b', 'LineWidth', 1.2); hold on;
xlim([0 1]);
xlabel('Frequency (Hz)'); ylabel('Amplitude');
title('呼吸率 FFT 频谱（CWT 0.1–0.5 Hz 滤波后）');
if ~isnan(RR_bpm)
    xline(RR_freq_hz, '--g', sprintf('%.1f BPM', RR_bpm), ...
        'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
    [~, idx_rr] = min(abs(f_rr - RR_freq_hz));
    plot(RR_freq_hz, amp_rr(idx_rr), 'gv', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
end

%% ── Step 10: 心率鲁棒计算（抑制呼吸高次谐波干扰后找峰）─────────────────
% 呼吸高次谐波（2f_RR, 3f_RR, ...）落在心率频段内会形成强干扰峰。
% 对 CWT 心率频谱在各谐波频点附近施加高斯衰减窗，再找峰。
%% ── CWT 卷积带通滤波：保留心率频带 0.5–3 Hz ──────────────────────────
% cwt() 只接受实数输入，因此对 I、Q 通道分别处理
HR_LOW  = 0.5;   % 心率频段下限 (Hz)
HR_HIGH = 3.0;   % 心率频段上限 (Hz)

% 1. 计算 CWT（默认使用 Morlet/amor 小波，频率单位与 fs 一致）
[wt_I, f_cwt] = cwt(I, fs);
[wt_Q,      ] = cwt(Q, fs);   % f_cwt 与上面相同

% 2. 构造频率掩膜，仅保留 0.5–3 Hz 的系数
mask = (f_cwt >= HR_LOW) & (f_cwt <= HR_HIGH);   % logical 列向量
wt_I(~mask, :) = 0;
wt_Q(~mask, :) = 0;

% 3. 逆 CWT 重建各通道，合成复数 IQ 信号
I_cwt = icwt(wt_I);
Q_cwt = icwt(wt_Q);
cwt_filtered = I_cwt(:) + 1i * Q_cwt(:);

% 4.  CWT 滤波后
t_vec = (1:length(I)) / fs;
[f_cwt2,P_cwt] = FFT_SHIFT(t_vec, cwt_filtered   - mean(cwt_filtered));


harmonic_bw    = 0.05;   % 谐波抑制半带宽 (Hz)
harmonic_depth = 0.1;    % 衰减后保留的幅值比例（0 = 完全抑制）
max_harmonic   = 6;      % 最高考虑到第 N 次谐波

amp_hr_robust = abs(P_cwt);   % 基于已有的心率 CWT FFT 谱

if ~isnan(RR_freq_hz)
    for k = 2:max_harmonic
        f_harm = k * RR_freq_hz;
        if f_harm > HR_HIGH, break; end
        % 高斯衰减：谐波频点处降至 harmonic_depth，远离时恢复为 1
        gauss_w = 1 - (1 - harmonic_depth) .* ...
            exp(-0.5 * ((f_cwt2 - f_harm) / harmonic_bw).^2);
        amp_hr_robust = amp_hr_robust .* gauss_w(:);
    end
end

valid_hr = (f_cwt2 >= HR_LOW) & (f_cwt2 <= HR_HIGH);
f_vhr    = f_cwt2(valid_hr);
a_vhr    = amp_hr_robust(valid_hr);

[~, locs_hr] = findpeaks(a_vhr, 'SortStr', 'descend', ...
    'MinPeakProminence', 0.1 * (max(a_vhr) - min(a_vhr)));

if isempty(locs_hr)
    warning('心率频段内（谐波抑制后）未检测到峰值。');
    HR_robust_bpm  = NaN;
    HR_robust_freq = NaN;
else
    HR_robust_freq = f_vhr(locs_hr(1));
    HR_robust_bpm  = HR_robust_freq * 60;
    fprintf('=== 鲁棒心率（谐波抑制）: %.1f BPM (%.3f Hz) ===\n', HR_robust_bpm, HR_robust_freq);
end

figure;
plot(f_cwt2, abs(P_cwt),    'r',   'LineWidth', 1.0, 'DisplayName', 'CWT HR 频谱（原始）'); hold on;
plot(f_cwt2, amp_hr_robust, 'b--', 'LineWidth', 1.4, 'DisplayName', '谐波抑制后频谱');
if ~isnan(RR_freq_hz)
    for k = 2:max_harmonic
        f_harm = k * RR_freq_hz;
        if f_harm > HR_HIGH, break; end
        xline(f_harm, ':k', sprintf('RR\\times%d', k), 'LabelVerticalAlignment', 'bottom');
    end
end
if ~isnan(HR_robust_bpm)
    xline(HR_robust_freq, '--b', sprintf('%.1f BPM', HR_robust_bpm), ...
        'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
    [~, idx_hr] = min(abs(f_cwt2 - HR_robust_freq));
    plot(HR_robust_freq, amp_hr_robust(idx_hr), 'bv', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
end
xlim([HR_LOW HR_HIGH]);
xlabel('Frequency (Hz)'); ylabel('Amplitude');
legend('Location', 'best');
title(sprintf('心率鲁棒估计（谐波抑制）: %.1f BPM', HR_robust_bpm));