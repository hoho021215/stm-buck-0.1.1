"""Plot CSV model samples and, when available, the parameters saved by run.py."""
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager, pyplot as plt
from matplotlib.ticker import MaxNLocator, MultipleLocator


def main():
    here = Path(__file__).resolve().parent
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else here / 'out'
    ko_font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if ko_font.exists():
        font_manager.fontManager.addfont(str(ko_font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(ko_font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'pdf.fonttype': 42, 'axes.spines.top': False,
                         'axes.spines.right': False, 'font.size': 10})

    with (out / 'waveforms.csv').open(newline='') as f:
        rows = list(csv.DictReader(f))
    series = {}
    for mask in ('6', '7'):
        points = [row for row in rows if row['feature_mask'] == mask]
        if not points:
            raise ValueError(f'No waveform samples for feature_mask={mask}')
        times = [float(row['relative_sample_time_us']) / 1000 for row in points]
        volts = [float(row['vout_model_V']) for row in points]
        series[mask] = times, volts

    # 구판 CSV에도 기록된 실제 입력전압을 사용합니다.
    vin_before = float(next(row['vin_V'] for row in rows if float(row['relative_sample_time_us']) < 0))
    vin_after = float(next(row['vin_V'] for row in rows if float(row['relative_sample_time_us']) >= 0))
    caption = f'폐루프 모의실험   |   입력 {vin_before:g} → {vin_after:g} V'
    sample_note = '선은 회로 모델의 출력전압이고, 펌웨어의 측정 통계에는 12비트 ADC 양자화가 포함됩니다.'
    params_file = out / 'parameters.json'
    model_note = ''
    if params_file.exists():
        params = json.loads(params_file.read_text())
        if params.get('firmware_version'):
            caption = f"{params['firmware_version']} {caption}"
        plant = params['plant']
        caption += f"   ·   저항 부하 {plant['load']:g} Ω   ·   PWM {1e-3 / params['pwm_period_s']:g} kHz"
        model_note = (f"모델 입력값   L = {plant['L'] * 1e6:g} µH, C = {plant['C'] * 1e6:g} µF, "
                      f"인덕터 직류저항 = {plant['dcr']:g} Ω, 커패시터 ESR = {plant['esr']:g} Ω")
        sample_note = (f"선은 PWM 주기 시작 {params['sample_at_s'] * 1e6:g} µs 뒤의 회로 모델 출력입니다. "
                       "펌웨어의 50 ms 통계에는 12비트 ADC 양자화가 포함됩니다.")

    fig, axs = plt.subplots(2, 1, figsize=(9.0, 6.7), sharey=True,
                            gridspec_kw={'height_ratios': [1.36, 1]})
    fig.subplots_adjust(left=.112, right=.97, bottom=.17, top=.825, hspace=.55)
    fig.text(.112, .957, '입력전압 변화에 대한 벅 컨버터 응답', size=19, weight='bold', va='top')
    fig.text(.112, .897, caption, size=10, color='#384659')
    for ax, lo, hi in ((axs[0], -2, 50), (axs[1], -.2, 2.1)):
        for mask, color, label in (('6', '#be5c13', '피드포워드 OFF (6)'),
                                   ('7', '#145781', '피드포워드 ON (7)')):
            times, volts = series[mask]
            ax.plot(times, volts, color=color, lw=1.55 if mask == '7' else 1.35, label=label)
        ax.axhline(15, color='#5b6776', lw=.85, ls='--', label='목표 15 V', zorder=0)
        ax.axvline(0, color='#7a8693', lw=.85, ls=':')
        ax.set_xlim(lo, hi)
        ax.set_ylabel('출력전압 (V)')
        ax.grid(axis='y', color='#dae0e4', lw=.6)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=6))
        ax.spines['left'].set_color('#82909b')
        ax.spines['bottom'].set_color('#82909b')
    axs[0].set_title('50 ms 측정 구간', loc='left', size=11, pad=7)
    axs[0].legend(frameon=False, ncol=3, loc='upper right', fontsize=8.5)
    axs[1].set_title('입력이 바뀐 직후의 2 ms 확대', loc='left', size=11, pad=7)
    axs[1].set_xlabel('입력 변화 후 시간 (ms)')
    axs[1].xaxis.set_major_locator(MultipleLocator(.25))
    fig.text(.112, .058, model_note, size=9, color='#384659')
    fig.text(.112, .030, sample_note, size=8.5, color='#384659')
    for ext in ('pdf', 'png'):
        fig.savefig(out / f'buck_line_step.{ext}', dpi=160, facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    main()
