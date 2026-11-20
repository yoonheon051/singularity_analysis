"""
명령줄 스크립트 검증 (ros2 run singularity_analysis <이름> 으로 실행하는 것들)

- visualize_results: 새 형식 CSV(특이점 종류 열 있음)·예전 형식 CSV(inf 포함) 모두 그림 저장, 최신 CSV 고르기
- calculate_jacobian / plot_ellipsoid / velocity_demo: 기본 URDF 로 끝까지 실행되고 결과 파일을 남기는지
Toolbox 는 conftest.toolbox_stub(ReferenceRobot)로 대신한다.
"""
import csv
import os
import sys

import numpy as np
import pytest

from conftest import NORMAL, needs_urdf
from singularity_analysis import singularity as sg

pytestmark = needs_urdf
pytest.importorskip('matplotlib')
pytest.importorskip('pandas')


@pytest.fixture
def results(monkeypatch, tmp_path):
    """결과 폴더를 임시 폴더로"""
    monkeypatch.setenv('SINGULARITY_RESULTS_DIR', str(tmp_path))
    return tmp_path


def run_main(module, args, monkeypatch, capsys):
    monkeypatch.setattr(sys, 'argv', [module.__name__] + list(args))
    module.main()
    return capsys.readouterr().out


def write_sweep_csv(path, robot, header_with_types=True, add_inf=False):
    """q5 를 0.6 → -0.6 으로 움직이는 동작(손목 특이점 통과)을 분석 노드와 같은 형식으로 기록"""
    cols = ['timestamp', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'manipulability', 'condition_number']
    if header_with_types:
        cols += ['singularity_type', 'sigma_min', 'wrist_idx', 'elbow_idx', 'shoulder_idx']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for k, q5 in enumerate(np.linspace(0.6, -0.6, 25)):
            q = [0, -0.3, 1.2, 0, q5, 0]
            a = sg.analyze(robot, q)
            row = [f'12:00:{k:02d}'] + q + [a['manipulability'], 'inf' if (add_inf and k == 12) else a['condition_number']]
            if header_with_types:
                i = a['indices']
                row += [sg.label(a['types']), a['sigma_min'], i['wrist'], i['elbow'], i['shoulder']]
            w.writerow(row)


def test_visualize_new_csv(results, robot, monkeypatch, capsys):
    from singularity_analysis import visualize_results as vr
    path = results / 'analysis_20261120_120000.csv'
    write_sweep_csv(path, robot)
    run_main(vr, [str(path)], monkeypatch, capsys)
    assert (results / 'analysis_20261120_120000_plot.png').stat().st_size > 10_000


def test_visualize_old_csv_with_inf(results, robot, monkeypatch, capsys):
    from singularity_analysis import visualize_results as vr
    path = results / 'analysis_log.csv'                      # 예전 버전 이름·형식 (종류 열 없음, inf 있음)
    write_sweep_csv(path, robot, header_with_types=False, add_inf=True)
    run_main(vr, [str(path)], monkeypatch, capsys)
    assert (results / 'analysis_log_plot.png').exists()


def test_latest_csv_picks_newest(results):
    from singularity_analysis import visualize_results as vr
    assert vr.latest_csv(str(results)).endswith('analysis_log.csv')      # 아무것도 없으면 예전 기본 이름
    old, new = results / 'analysis_20261101_090000.csv', results / 'analysis_20261120_090000.csv'
    old.write_text('x'); new.write_text('x')
    os.utime(old, (1, 1))
    (results / 'notes.csv').write_text('x')                               # analysis_ 로 시작하지 않으면 무시
    assert vr.latest_csv(str(results)) == str(new)


def test_calculate_jacobian_prints_type(results, toolbox_stub, monkeypatch, capsys):
    from singularity_analysis import calculate_jacobian as cj
    out = run_main(cj, ['--q', '0', '0.5', '0.05', '0', '1', '0'], monkeypatch, capsys)
    assert 'Singularity type: elbow' in out
    out = run_main(cj, ['--q'] + [str(v) for v in NORMAL], monkeypatch, capsys)
    assert 'Singularity type: normal' in out


def test_calculate_jacobian_missing_file_exits(toolbox_stub, monkeypatch, capsys):
    from singularity_analysis import calculate_jacobian as cj
    with pytest.raises(SystemExit):
        run_main(cj, ['no_such_robot.urdf'], monkeypatch, capsys)


def test_plot_ellipsoid_default_poses(results, toolbox_stub, monkeypatch, capsys):
    from singularity_analysis import plot_ellipsoid as pe
    out = run_main(pe, ['--no-show'], monkeypatch, capsys)
    kinds = [line.split(']')[1].split()[0] for line in out.splitlines() if line.startswith('[')]
    assert kinds == ['normal', 'wrist', 'elbow', 'shoulder']   # 기본 4자세가 각각 의도한 종류로 분류
    assert (results / 'manipulability_ellipsoids.png').exists()


def test_velocity_demo_runs(results, toolbox_stub, monkeypatch, capsys):
    from singularity_analysis import velocity_demo as vd
    out = run_main(vd, ['--no-show'], monkeypatch, capsys)
    assert 'Inverse' in out and 'DLS' in out
    assert (results / 'velocity_dls_demo.png').exists()
