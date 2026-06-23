

Inspire_	그리퍼 컨트롤러 및 리타켓팅 데이터, 현재 각도 값을 system_right 로 전달할 토픽 생성 및 퍼블리쉬 필요
  1. 인스파이어 컨트롤러 : 시리얼 통신 / 연결된 USB 포트 권한 설정 필요 ""sudo chmod 777 /dev/ttyUSB0""
   ros2 run inspier_driver inspire_driver_node
  2. 인스파이어 리타겟팅 노드 : 마찬가지로 마누스 노드 우선 실행 권장
   ros2 run inspire_driver inspire_bridge_node


System_		UI 및 데이터 수집 및 저장 노드
	UI 실행 : Left, Right 구분되어 있음 방향에 혼동하지 않도록 주의
	ros2 run system_ui system_ui_node
	
	패키지 설명
	 system_ui
	   systemUi.py 는 ui생성, 이벤트 생성 전용
	   node_manager.py 는 이벤트 받아서 필요한 Node들을 새로운 process 에서 실행시킴
	 system_interface 는 커스텀 메시지 생성을 위한 패키지
	 system_right / system_left 는 각 side 시스템에서 그리퍼, 로봇의 최신 데이터들을 받고, ui 요청을 실행하는 패키지로,
	 			해당 패키지를 통해 데이터 저장 및 출력이 가능하도록 함.
	 data_exporter 는  수신 받은 데이터를 바로 저장하는 패키지로 노드 실행 시 Record/ 에 파일을 생성하고 해당 파일에 데이터들을 바로 작성한다.
	 파일이 있는 상태에서 실행 시 중복되지 않도록 0, 1, 2, 3... 과 같이 카운트로 구분되어 새로 생성된다.



 필요 환경 설정
  IP 설정 : 현재 로봇 시스템은 192.168.4.xxx 대역 IP 사용 중이며, 리눅스 전원을 켰을 때 IP 설정이 필요함.
    sudo ip addr flush dev enp4s0   : 현재 사용중인 enp4s0 ip 삭제
    sudo ip addr add 192.168.4.xxx/24 dev enp4s0  enp4s0 새로운 IP 부여
    sudo route add -net 192.168.4.0 netmask 255.255.255.0 dev enp4s0  라우팅
    ifconfig 로 inet 활성화 및 IP address 확인하기
   권한 설정
       인스파이어 연결에 필요한 USB 포트 권한 설정하기
       보통의 경우 /dev/ttyUSB0 으로 주로 연결됨.
    sudo chmod 777 /dev/ttyUSB0		: /dev/ttyUSB0 읽기/쓰기 권한 다 주기
    sudo chmod 777 /dev/ttyUSB1		: /dev/ttyUSB1 읽기/쓰기 권한 다 주기
    echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer		통신 속도 증가
   소싱
    빌드 및 코드에 변경이 있을 시, 새로 빌드 후 소싱하는 과정이 필요하며 이는 터미널이 새로 생길때마다 필요한 과정
    source install/setup.bash
       모든 커맨드는 ~/system_Teleop 경로에서 ( hyundae/seoul_uiwang/system_Teleop$ )



##### 인스파이어 필수 설정 #####
환경 세팅 함수
sudo ip addr flush dev enp4s0							// 기존 IP 삭제
sudo ip addr add 192.168.4.55/24 dev enp4s0					// IP 변경

sudo chmod 777 /dev/ttyUSB*							// USB 포트 권한
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer		// 통신 속도
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer		// 통신 속도


