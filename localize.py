import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from kf import *

mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
initialized = False
prev_yaw = 0
state = np.zeros(5)
cov = np.eye(5)
prev_t = 0
while True:
  time.sleep(0.1)
  try:
    data = mcu.readline()
    if data:
      data = data.decode('utf-8').split()
      if len(data) < 5:
        continue

      data = [float(d) for d in data]

      if not initialized:
        prev_yaw = data[0]
        state, cov = init_state_given_yaw(np.pi, data[1:])
        initialized = True
        prev_t = time.time()
        continue

      t = time.time()
      yaw = data[0]
      state, cov = predict(state, cov, yaw - prev_yaw, t - prev_t)
      state, cov = correct(state, cov, data[1:])
      print(state)
      prev_t = t
      prev_yaw = yaw


    mcu.flushInput()

  except Exception as e:
    print(e)
    mcu.close()
    break

