# 串口监视器：COM8 @115200，Ctrl+C 停止
import serial

s = serial.Serial('COM8', 115200, timeout=1)
print('正在监视 COM8 @115200（Ctrl+C 停止）...')
while True:
    line = s.readline().decode('utf-8', 'replace').strip()
    if line:
        print(line, flush=True)
