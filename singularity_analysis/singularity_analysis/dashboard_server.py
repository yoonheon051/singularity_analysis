"""
실시간 특이점 대시보드 웹 서버 (ros2 run singularity_analysis dashboard)

dashboard/index.html 을 HTTP 로 띄워 주기만 한다. 데이터는 브라우저가 rosbridge(WebSocket, 기본 9090)에서
/singularity/status 를 직접 받는다. 그래서 이 서버는 ROS 없이 파이썬 표준 라이브러리만 쓴다.

    ros2 run singularity_analysis dashboard              # http://<이 PC IP>:8080
    ros2 run singularity_analysis dashboard --port 8000

같은 네트워크의 태블릿·폰에서도 http://<이 PC IP>:8080 으로 열 수 있다 (rosbridge 주소는 자동으로 같은 호스트의 9090).
"""
import argparse
import functools
import http.server
import os
import socket
import sys


def dashboard_dir():
    """index.html 이 있는 폴더: 설치된 패키지 share → 저장소의 singularity_analysis/dashboard"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('singularity_analysis'), 'dashboard')
        if os.path.isfile(os.path.join(path, 'index.html')):
            return path
    except Exception:   # ROS 가 없거나 아직 빌드하지 않음
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'dashboard'))


def local_ip():
    """다른 기기에서 접속할 때 쓸 이 PC 의 IP (모르면 localhost)"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))   # 실제로 패킷을 보내지는 않음
            return s.getsockname()[0]
    except OSError:
        return 'localhost'


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):   # 요청마다 터미널에 찍지 않음
        pass

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')   # 페이지를 고치면 새로고침만으로 반영
        super().end_headers()


def make_server(host, port, directory):
    handler = functools.partial(QuietHandler, directory=directory)
    return http.server.ThreadingHTTPServer((host, port), handler)


def main(argv=None):
    ap = argparse.ArgumentParser(description='M0609 특이점 대시보드 웹 서버')
    ap.add_argument('--host', default='0.0.0.0', help='열어 둘 주소 (기본: 모든 네트워크)')
    ap.add_argument('--port', type=int, default=8080, help='HTTP 포트 (기본 8080)')
    args, _ = ap.parse_known_args(sys.argv[1:] if argv is None else argv)   # launch 가 붙이는 --ros-args 는 무시

    directory = dashboard_dir()
    if not os.path.isfile(os.path.join(directory, 'index.html')):
        sys.exit(f'대시보드 파일을 찾을 수 없습니다: {directory}/index.html')

    server = make_server(args.host, args.port, directory)
    print(f'대시보드: http://localhost:{args.port}  (다른 기기: http://{local_ip()}:{args.port})', flush=True)
    print('데이터는 rosbridge(ws://<같은 호스트>:9090)의 /singularity/status 에서 받습니다.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
