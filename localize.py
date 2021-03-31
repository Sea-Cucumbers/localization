import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from kf import *

mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
initialized = False
prev_yaw = 0

nfilters = 8
states = np.zeros((5, nfilters))
covs = np.array([np.eye(5) for i in range(nfilters)])
log_weights = np.log(np.ones(nfilters)/nfilters)
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

        for i in range(nfilters):
          states[:, i], covs[i] = init_state_given_yaw(i*np.pi/4, data[1:])

        initialized = True
        prev_t = time.time()
        continue

      t = time.time()
      yaw = data[0]

      for i in range(nfilters):
        states[:, i], covs[i] = predict(states[:, i], covs[i], yaw - prev_yaw, t - prev_t)
        states[:, i], covs[i], log_weights[i] = correct(states[:, i], covs[i], data[1:], log_weights[i])

      log_weights = normalize_log_weights(log_weights)

      live_filters = log_weights > -10
      states = states[:, live_filters]
      covs = covs[live_filters]
      log_weights = normalize_log_weights(log_weights[live_filters])
      nfilters = len(covs)

      state = np.matmul(states, np.exp(log_weights))
      state_deg = np.copy(state)
      state_deg[2] *= 180/np.pi
      print(np.trunc(state_deg))
      prev_t = t
      prev_yaw = yaw
    else:
      print('No data')

    mcu.flushInput()

  except Exception as e:
    print(e)
    mcu.close()
    break

