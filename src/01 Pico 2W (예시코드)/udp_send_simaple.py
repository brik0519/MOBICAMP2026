# udp_send_simple.py
# Pico 2 W에서 PC로 UDP 메시지를 보내는 단순한 예제

import network
import socket
import time

#SSID = "YOUR_WIFI_SSID"
#PASSWORD = "YOUR_WIFI_PASSWORD"

SSID = "RiaSummer2G"
PASSWORD = "730124go"

PC_IP = "192.168.2.3"   # PC의 IPv4 주소로 수정
PC_PORT = 5005

# 1. Wi-Fi 공유기에 연결 ------------------------------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print("Connecting to Wi-Fi...")
while not wlan.isconnected():
    time.sleep(0.2)
print("Wi-Fi connected")
print("Pico IP:", wlan.ifconfig()[0])

# 2. UDP 소켓을 생성 -----------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

count = 0

while True:
    message = "hello,{}".format(count)
    # 3. 문자열을 PC로 전송 ---------------------------
    sock.sendto(message.encode(), (PC_IP, PC_PORT))
    print("sent:", message)
    count += 1
    time.sleep(1)