import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

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
"""

def plot_singularity_metrics(csv_path):
    # 1. 파일 존재 여부 확인(예외처리)
    if not os.path.exists(csv_path):
        print(f"에러: {csv_path} 파일을 찾을 수 없습니다.")
        return
    
    # 2. 데이터 로드: Pandas를 이용해 CSV를 구조화된 데이터프레임으로 변환
    df = pd.read_csv(csv_path)

    # 3. 캔버스 설정: 2개의 그래프를 세로로 배치(2행 1열)
    # figsize: 그래프 크기, sharex: X축(시간 / 샘플 수)을 두 그래프가 공유하도록 설정
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
    ax2.set_xlabel('Sample Count (Time Step)')           # X축 라벨: 데이터 수집 순서
    ax2.set_ylabel('Condition Number (Log Scale)')       # Y축 라벨
    ax2.set_title('Jacobian Condition Number Over Time')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(loc='upper left')   # 오른쪽 위는 최근 특이점 표시(X)와 겹칠 수 있어 왼쪽에

    # 4. 레이아웃 최적화: 그래프 간 간격 자동 조정 및 화면 출력
    plt.tight_layout()

    # 5. 그래프를 CSV와 같은 results/ 폴더에 이미지로 저장 (화면이 없는 환경에서도 결과가 남도록)
    png_path = os.path.join(os.path.dirname(csv_path), 'singularity_analysis_plot.png')
    plt.savefig(png_path, dpi=150)
    print(f"그래프 저장: {png_path}")

    plt.show()

# 파이썬 스크립트 실행 엔트리 포인트
if __name__ == "__main__":

    # 데이터 수집 결과인 analysis_log.csv 경로 지정 (실행 위치와 상관없이 저장소의 results/ 폴더)
    # 다른 파일을 보려면 인자로 경로를 준다: python3 scripts/visualize_results.py 다른.csv
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    csv_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(repo_dir, 'results', 'analysis_log.csv')
    plot_singularity_metrics(csv_file)