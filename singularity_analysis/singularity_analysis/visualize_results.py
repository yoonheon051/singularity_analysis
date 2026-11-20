import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

try:                                  # ros2 run / 패키지로 설치했을 때
    from . import singularity as sg
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import singularity as sg   # 같은 폴더: 특이점 종류 임계값

"""
[ 로봇 기구학 성능 지표 시각화 가이드 ]

1. 가동성 지수 (Manipulability Index, w)
   - 의미: 로봇이 현재 자세에서 모든 방향으로 얼마나 자유롭게 움직일 수 있는가?
   - 수치: 0에 가까워지면 '특이점(Singularity)' 상태로, 특정 방향으로의 이동이 물리적으로 불가능해짐.
   - 분석: 그래프의 골짜기(V자)는 로봇이 작업 중 가장 부자유스러운 지점을 의미함.

2. 조건수 (Condition Number)
   - 의미: 자코비안 행렬의 수치적 안정성 (최대 특이값 / 최소 특이값).
   - 수치: 1이면 모든 방향으로 균일한 속도를 내는 '이상적 상태(Isotropic)', 커질수록 제어 불안정.
   - 분석: 그래프의 피크(A자)는 제어 오차가 증폭되어 로봇이 튀거나 멈출 수 있는 위험 구간임.

3. 특이점 종류 (새 CSV에만 있음, singularity.py 참고)
   - wrist / elbow / shoulder 지표: 0에 가까울수록 그 종류의 특이점. 점선(임계값) 아래면 위 그래프 배경을 색칠.
"""

# 특이점 종류별 색 (그래프 배경·지표 선)
TYPE_COLORS = {'wrist': 'tab:orange', 'elbow': 'tab:purple', 'shoulder': 'tab:cyan'}


def plot_singularity_metrics(csv_path):
    # 1. 파일 존재 여부 확인(예외처리)
    if not os.path.exists(csv_path):
        print(f"에러: {csv_path} 파일을 찾을 수 없습니다.")
        return
    
    # 2. 데이터 로드: Pandas를 이용해 CSV를 구조화된 데이터프레임으로 변환
    df = pd.read_csv(csv_path)

    # 3. 캔버스 설정: 그래프를 세로로 배치
    # 특이점 종류 열(singularity_type, *_idx)이 있는 새 CSV면 3행, 예전 CSV면 2행
    # figsize: 그래프 크기, sharex: X축(시간 / 샘플 수)을 그래프들이 공유하도록 설정
    has_types = {'singularity_type', 'wrist_idx', 'elbow_idx', 'shoulder_idx'} <= set(df.columns)
    if has_types:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 13), sharex=True)
    else:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # x_axis: 수집된 데이터의 개수만큼 인덱스 생성(시간의 흐름)
    x_axis = range(len(df))

    ###############################################
    # --- 상단 그래프: 가동성 지수(Manipulability) --- #
    ###############################################

    # .value를 사용하여 Numpy배열로 변환함으로써 최신 Pandas의 인덱싱 호환성 에러를 방지
    ax1.plot(x_axis, df['manipulability'].values, color='blue', linewidth=2, label='Manipulability Index')
    
    # 가동성 지표의 임계값(0.01)표시: 이 선 아래로 떨어지면 특이점 위험 경고
    ax1.axhline(y=0.01, color='red', linestyle=':', label='Critical Threshold')    
    
    ax1.set_ylabel('Index Value (w)')                    # Y축 라벨: 가동성 수치
    ax1.set_title('Robot Manipulability Over Time')      # 그래프 제목
    ax1.grid(True, linestyle='--', alpha=0.7)            # 격자 표시
    ax1.legend()                                         # 범례 표시

    ##############################################
    # --- 하단 그래프: 조건수 (Condition Number) --- #
    ##############################################

    # 조건수는 특이점 부근에서 값이 기하급수적으로 커지므로 로그 스케일(Log Scale)로 시각화
    cond = df['condition_number'].astype(float).values
    ax2.plot(x_axis, cond, color='green', linewidth=2, label='Condition Number')

    # 무한대(완전 특이점)는 로그 그래프에 그려지지 않아 사라지므로, 그래프 맨 위에 빨간 X로 따로 표시
    singular = np.isinf(cond)
    if singular.any():
        finite = cond[np.isfinite(cond)]
        top = finite.max() * 10 if finite.size else 1e3
        ax2.scatter(np.flatnonzero(singular), np.full(singular.sum(), top), color='red', marker='x', s=40,
                    zorder=3, label=f'Singular (cond = inf) x{singular.sum()}')   # 기본 글꼴에 한글이 없어 영어로

    ax2.set_yscale('log')                                # Y축을 로그 스케일로 설정 (중요)
    if not has_types:
        ax2.set_xlabel('Sample Count (Time Step)')       # X축 라벨: 데이터 수집 순서 (맨 아래 그래프에만)
    ax2.set_ylabel('Condition Number (Log Scale)')       # Y축 라벨
    ax2.set_title('Jacobian Condition Number Over Time')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(loc='upper left')   # 오른쪽 위는 최근 특이점 표시(X)와 겹칠 수 있어 왼쪽에

    if has_types:
        ###########################################################
        # --- 위 두 그래프 배경: 어떤 특이점 근처였는지 구간 색칠 --- #
        ###########################################################
        kinds = df['singularity_type'].fillna('normal').astype(str).values
        start = 0
        shaded = set()
        for i in range(1, len(kinds) + 1):
            if i == len(kinds) or kinds[i] != kinds[start]:
                kind = kinds[start]
                if kind != 'normal':
                    color = TYPE_COLORS.get(kind, 'gray')   # 두 종류가 겹치면 회색
                    for ax in (ax1, ax2, ax3):
                        ax.axvspan(start - 0.5, i - 0.5, color=color, alpha=0.15, lw=0,
                                   label=None if (kind in shaded or ax is not ax1) else f'near {kind}')
                    shaded.add(kind)
                start = i
        if shaded:
            ax1.legend()

        ##################################################
        # --- 하단 그래프: 특이점 종류별 지표 (0이면 특이점) --- #
        ##################################################
        for kind, limit in (('wrist', sg.WRIST_LIMIT), ('elbow', sg.ELBOW_LIMIT), ('shoulder', sg.SHOULDER_LIMIT)):
            ax3.plot(x_axis, df[f'{kind}_idx'].astype(float).values, color=TYPE_COLORS[kind], linewidth=2,
                     label=f'{kind} index')
            ax3.axhline(y=limit, color=TYPE_COLORS[kind], linestyle=':', linewidth=1)
        ax3.set_ylim(bottom=0)
        ax3.set_ylabel('Index (0 = singular)')
        ax3.set_title('Singularity Type Indices (dotted: threshold)  wrist=|det[z4 z5 z6]|, '
                      'elbow=sin(c2,c3), shoulder=out-of-plane c1 / arm length')
        ax3.set_xlabel('Sample Count (Time Step)')
        ax3.grid(True, linestyle='--', alpha=0.7)
        ax3.legend(loc='upper right')

    # 4. 레이아웃 최적화: 그래프 간 간격 자동 조정 및 화면 출력
    plt.tight_layout()

    # 5. 그래프를 CSV와 같은 폴더에 CSV 이름을 딴 이미지로 저장 (화면이 없는 환경에서도 결과가 남도록)
    png_path = os.path.splitext(csv_path)[0] + '_plot.png'
    plt.savefig(png_path, dpi=150)
    print(f"그래프 저장: {png_path}")

    plt.show()


def latest_csv(results_dir):
    """results/ 에서 가장 최근에 기록된 분석 CSV (실행마다 analysis_날짜_시각.csv, 예전 버전은 analysis_log.csv)"""
    files = [os.path.join(results_dir, f) for f in os.listdir(results_dir)
             if f.startswith('analysis_') and f.endswith('.csv')] if os.path.isdir(results_dir) else []
    return max(files, key=os.path.getmtime) if files else os.path.join(results_dir, 'analysis_log.csv')


def main():
    # 인자가 없으면 결과 폴더(~/singularity_results 또는 SINGULARITY_RESULTS_DIR)에서 가장 최근 실행의 CSV 를 그림
    # 다른 파일을 보려면 인자로 경로를 준다: visualize_results ~/singularity_results/analysis_20261120_153000.csv
    csv_file = sys.argv[1] if len(sys.argv) > 1 else latest_csv(sg.results_dir())
    print(f"그릴 파일: {csv_file}")
    plot_singularity_metrics(csv_file)


# 파이썬 스크립트 실행 엔트리 포인트
if __name__ == "__main__":
    main()
