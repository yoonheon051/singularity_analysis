import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_singularity_metrics(csv_path):
    # 1. CSV 데이터 로드
    if not os.path.exists(csv_path):
        print(f"에러: {csv_path} 파일을 찾을 수 없습니다.")
        return
    
    df = pd.read_csv(csv_path)

    # 2. 그래프 설정 (2행 1열 구성)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # x축: 데이터 인덱스 또는 시간 (데이터가 많을 경우 인덱스가 깔끔합니다)
    x_axis = range(len(df))

    # 3. 가동성 지수 (Manipulability) 그래프
    ax1.plot(x_axis, df['manipulability'], color='blue', linewidth=2, label='Manipulability Index')
    ax1.set_ylabel('Index Value (w)')
    ax1.set_title('Robot Manipulability Over Time')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()
    # 0에 가까울수록 위험함을 표시하는 가이드라인
    ax1.axhline(y=0.01, color='red', linestyle=':', label='Critical Threshold')

    # 4. 조건수 (Condition Number) 그래프
    # 조건수는 값이 너무 커질 수 있으므로 로그 스케일로 보는 것이 유리할 수 있습니다.
    ax2.plot(x_axis, df['condition_number'], color='green', linewidth=2, label='Condition Number')
    ax2.set_xlabel('Sample Count')
    ax2.set_ylabel('Condition Number (log scale)')
    ax2.set_yscale('log') # 수치 변화가 크므로 로그 스케일 적용
    ax2.set_title('Jacobian Condition Number Over Time')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()

    plt.tight_layout()
    plt.show()

# 파일 경로 지정 및 실행
if __name__ == "__main__":
    csv_file = 'results/analysis_log.csv'
    plot_singularity_metrics(csv_file)