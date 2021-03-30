import serial
import time
import matplotlib.pyplot as plt
import numpy as np

mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
yaws = []
dists = []
while True:
  time.sleep(0.1)
  try:
    data = mcu.readline()
    if data:
      data = data.decode('utf-8').split()
      if len(data) < 5:
        continue

    mcu.flushInput()

  except Exception as e:
    print(e)
    mcu.close()
    break

