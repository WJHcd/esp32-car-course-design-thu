# 构建引导脚本（在 Git Bash/MSYS 下构建 ESP-IDF 工程的桥接）
#
# 背景：idf.py 检测到 'MSYSTEM' 环境变量存在时只打印警告、不执行构建，
# 而 Git Bash 的 MSYS 运行时会向所有子进程自动注入 MSYSTEM，unset/env -u
# 均无效。本脚本用【原生 Windows python】先弹出 MSYSTEM 等 MSYS 变量，
# 再以子进程启动 idf.py —— 原生进程的子进程继承我们显式给定的环境块，
# MSYS 运行时不再插手，idf.py 得以正常运行。
#
# 用法：python _build.py set-target esp32s3
#       python _build.py build
import os
import sys
import subprocess

for k in ('MSYSTEM', 'MINGW_PREFIX', 'MINGW_CHOST', 'MSYSTEM_CARCH',
          'MINGW_PACKAGE_PREFIX'):
    os.environ.pop(k, None)

os.environ['IDF_PATH'] = r'C:\esp\v5.4.4\esp-idf'
os.environ['IDF_TOOLS_PATH'] = r'C:\Espressif\tools'
os.environ['IDF_PYTHON_ENV_PATH'] = r'C:\Espressif\tools\python\v5.4.4\venv'
os.environ['IDF_PYTHON_CHECK_CONSTRAINTS'] = '0'
# esp_rom 生成 gdbinit 需要（export 脚本通常会设置；路径须以分隔符结尾）
os.environ['ESP_ROM_ELF_DIR'] = r'C:\Espressif\tools\esp-rom-elfs\20241011' + os.sep

# 仅保留 Windows 风格工具路径（EIM 无 tools.json，需手动给出，同 build.bat）
os.environ['PATH'] = (
    r'C:\Espressif\tools\cmake\3.30.2\bin;'
    r'C:\Espressif\tools\ninja\1.12.1;'
    r'C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;'
    r'C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;'
    r'C:\Espressif\tools\python\v5.4.4\venv\Scripts;'
    r'C:\Windows\System32;C:\Windows'
)

idf = os.path.join(os.environ['IDF_PATH'], 'tools', 'idf.py')
ret = subprocess.call([sys.executable, idf] + sys.argv[1:], env=os.environ)
sys.exit(ret)
