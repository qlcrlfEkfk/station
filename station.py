import socket
import time
from threading import Thread, Lock
from pymycobot.mycobot import MyCobot
from pymodbus.client import ModbusSerialClient

# Modbus RTU 클라이언트 초기화
plc_client = ModbusSerialClient(
    # method='rtu',
    port='com4',
    baudrate=9600,
    timeout=2,
    parity='N',
    stopbits=1,
    bytesize=8
)

mc = MyCobot('com3',115200)

current_stage = 0  # 현재 단계 (0: 대기, 1: 분리 작업, 2: 마커 확인, 3: 부착 작업)
stage_lock = Lock()

server_host = '0.0.0.0'  # PC의 서버 주소
server_port = 5000        # PC에서 듣는 포트
agv_ip = '172.30.1.14'   # AGV의 IP 주소
agv_port = 6000          # AGV의 수신 포트

# RS-485로 PLC 신호 전송
def send_rs485_signal_modbus(register_address, value):
    """
    Modbus를 통해 PLC에 신호 전송
    Args:
        register_address (int): PLC 레지스터 주소
        value (int): PLC에 쓸 값
    """
    try:
        if plc_client.connect():
            write_result = plc_client.write_register(register_address, value, slave=1)
            if write_result.isError():
                print(f"D00000 쓰기 실패: {write_result}")
            else:
                print(f"D00000에 {register_address}에 {value} 전송 성공")
        else:
            print("PLC 연결 실패")
    except Exception as e:
        print(f"PLC 신호 전송 에러 메세지: {e}")
    finally:
        plc_client.close()
 
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
    print("마커 작업 시작")
    mc.set_basic_output(1,0) # p3 on
    time.sleep(4)
    marker_positions = [
        [0, 0, 0, 0, 0, 0],
        [-15.73, -37.52, -75.32, -61.96, 165, -175.16],
        [0, 0, 0, 0, 0, 0],
        [1.4, -68.2, -23.55, -54.66, 5.62, -30.49],
        [0, 0, 0, 0, 0, 0]
    ]
    move_cobot_to_positions(marker_positions)

    print("AGV가 마커 분리 그리고 마커가 컨베이어 위에 위치")
    mc.set_basic_output(1, 1)  # P3 off
    
# 사람이 놓든 미끄러져 내려오든 컨베이어에 마커 놓기 완료
def perform_marker_check():
    print("컨베이어에 마커가 있는지 확인 중...")
    marker_input = mc.get_basic_input(1) 
    if marker_input ==0:
        print("컨베이어에 마커 위치 완료")
        return True
    return False

# 부착 작업
def perform_attachment_task():
    print("부착 작업중...")
       
    mc.set_basic_output(1,0) # p4
    time.sleep(4)

    attachment_positions = [
        [0, 0, 0, 0, 0, 0],
        [1.4, -68.2, -23.55, -54.66, 5.62, -30.49],
        [0, 0, 0, 0, 0, 0],
        [-15.73, -37.52, -75.32, -61.96, 165, -175.16],
        [0, 0, 0, 0, 0, 0]
    ]
    move_cobot_to_positions(attachment_positions)

    print("부착 작업 완료")
    mc.set_basic_output(1,1) # p4
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
                send_rs485_signal_modbus(77)
                time.sleep(4)
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

