import socket
import time
from threading import Thread, Lock
from pymycobot.mycobot import MyCobot
import serial

# RS-485 포트 설정 
plc_port = serial.Serial(
    port = 'com10', # 포트 설정
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

mc = MyCobot('com3',115200)

current_stage = 0  # 현재 단계 (0: 대기, 1: 분리 작업, 2: 마커 확인, 3: 부착 작업)
stage_lock = Lock()

server_host = '0.0.0.0'  # PC의 서버 주소
server_port = 5000        # PC에서 듣는 포트
agv_ip = '172.30.1.35'   # AGV의 IP 주소
agv_port = 6000          # AGV의 수신 포트

# RS-485로 PLC 신호 전송
def send_rs485_signal(signal):
    """
    RS-485로 PLC 신호 전송
    Args:
        signal (int): PLC로 보낼 신호
    """
    try:
        data = f"{signal}\n".encode('utf-8')  # 문자열로 변환 후 바이트 인코딩
        plc_port.write(data)  # ASCII 데이터 전송
        print(f"PLC에 보내는 ASCII 신호: {data.decode('utf-8').strip()}")
    except Exception as e:
        print(f"PLC에 보내는 ASCII 신호 에러 메세지: {e}")
 
# 코봇 위치를 이동
def move_cobot_to_positions(positions, speed=20, delay =5):
    """
    코봇 관절 위치를 주어진 각도로 이동.
    Args:
        positions (list): 각도 리스트 (각 상태 정의)
        speed (int): 속도 (기본값 20)
        delay (float): 각 상태에서 대기 시간 (기본값 5초)
    """
    for angles in positions:
        mc.send_angles(angles, speed)
        time.sleep(delay)

#AGV에서 마커 분리 작업 및 컨베이어에 옮기는 작업
def perform_marker_task():
    print("starting marker task")
    marker_positions = [
        [0, 0, 0, 0, 0, 0],
        [-15.73, -37.52, -75.32, -61.96, 165, -175.16],
        [0, 0, 0, 0, 0, 0],
        [1.4, -68.2, -23.55, -54.66, 5.62, -30.49],
        [0, 0, 0, 0, 0, 0]
    ]
    move_cobot_to_positions(marker_positions)

    mc.set_basic_output(1,0) # p3 on
    print("AGV가 마커 분리 그리고 마커가 컨베이어 위에 위치")
    time.sleep(4)

    mc.set_basic_output(1, 1)  # P3 off
    print("p3 신호 끝")
    time.sleep(4)

# 사람이 놓든 미끄러져 내려오든 컨베이어에 마커 놓기 완료
def perform_marker_check():
    print("컨베이어에 마커가 있는지 확인 중...")
    marker_input = mc.get_basic_input(1) # p3이 코봇에 신호 보내는 중
    if marker_input ==0:
        print("컨베이어에 마커 위치 완료")
        return True
    return False

# 부착 작업
def perform_attachment_task():
    print("부착 작업중...")
       
    mc.set_basic_output(2,0) # p4
    time.sleep(4)
    attachment_positions = [
        [0, 0, 0, 0, 0, 0],
        [1.4, -68.2, -23.55, -54.66, 5.62, -30.49],
        [0, 0, 0, 0, 0, 0],
        [-15.73, -37.52, -75.32, -61.96, 165, -175.16],
        [0, 0, 0, 0, 0, 0]
    ]
    move_cobot_to_positions(attachment_positions)

    mc.set_basic_output(2,1) # p4
    print("부착 작업 완료")
    time.sleep(4)

# AGV에 신호 보내기
def send_signal_to_agv():
    """작업 완료 후 AGV에 신호 전송"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as agv_socket:
            agv_socket.connect((agv_ip, agv_port))  # AGV의 IP와 포트
            agv_socket.sendall(b"Task completed. AGV ready to proceed")
            print("AGV에 신호 전송 완료")
    except Exception as e:
        print(f"AGV에 신호 전송 에러 메세지: {e}")

# 작업 스레드
def process_tasks():
    global current_stage
    while True:
        with stage_lock:
            # 작업 단계 시작
            if current_stage == 1:
                send_rs485_signal(485)
                perform_marker_task()
                current_stage = 2

            elif current_stage == 2:
                if perform_marker_check():
                    current_stage = 3
                else:
                    print("마커 탐지 안됨 신호 기다리는 중")
                    time.sleep(2)

            elif current_stage == 3:
                perform_attachment_task()
                send_signal_to_agv()
                print("모든 작업 완료 초기화 중")
                current_stage = 0
        time.sleep(0.1)

# 소켓 서버
def start_socket_server():
    global current_stage
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((server_host, server_port))
        s.listen(1)
        print("AGV 신호 수신 중")
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                data = conn.recv(1024)
                if data:
                    print(f"Received signal: {data.decode('utf-8')}")
                    with stage_lock:
                        if current_stage ==0:
                            current_stage = 1
                            print("작업 스테이지 1로 업데이트")

# 메인 루프
def main():

    #작업 스레드 시작
    task_thread = Thread(target=process_tasks)
    task_thread.start()

    # 소켓 서버 쓰레드 시작
    server_thread = Thread(target=start_socket_server)
    server_thread.start()

    try:
        task_thread.join()
        server_thread.join()

    except KeyboardInterrupt:
        print("시스템 닫는 중")

    finally:
        print("시스템 닫는 중")

if __name__ == '__main__':
    main()

