from angle_mod import *
import numpy as np

def init_state_given_yaw(yaw, obs):
  state = np.zeros(5)
  cov = np.eye(5)
  state[2] = yaw
  if yaw >= 7*np.pi/4 or yaw <= np.pi/4:
    # Facing forward
    c = np.cos(yaw)
    state[0] = obs[3]*c
    state[1] = obs[0]*c

  elif state[2] >= 3*np.pi/8 and state[2] <= 5*np.pi/8:
    # Facing left
    c = np.cos(yaw - np.pi/2)
    state[0] = obs[2]*c
    state[1] = obs[3]*c

  elif state[2] >= 7*np.pi/8 and state[2] <= 9*np.pi/8:
    # Facing backward
    c = np.cos(yaw - np.pi)
    state[0] = obs[1]*c
    state[1] = obs[2]*c

  elif state[2] >= 11*np.pi/8 and state[2] <= 13*np.pi/8:
    # Facing right
    c = np.cos(yaw - 3*np.pi/2)
    state[0] = obs[0]*c
    state[1] = obs[1]*c

  return state, cov

# state is [x, y, yaw, vx, vy].
# If we're facing long side of guiderail, theta = 0. (x, y) = (0, 0)
# is at the intersection of guiderails. +x is left, +y is backwards.
# theta is in radians, x and y are in cm, vx and vy are in meters per second.
# Front of the robot is sensor 0. Positive yaw velocity
# is about the vertical axis
def predict(state, cov, dyaw, dt):
  F_t = np.eye(5)
  F_t[0, 3] = dt
  F_t[1, 4] = dt
  state = np.matmul(F_t, state)
  state[2] += dyaw
  state[2] = angle_mod(state[2])

  Q = np.eye(5)
  cov = np.matmul(np.matmul(F_t, cov), F_t.transpose()) + Q
  return state, cov

# Sensor 0 is on the same side as the ethernet cable. 1-3 go counterclockwise
def correct(state, cov, obs):
  R = 1000*np.eye(4)
  zhat = 200*np.ones(4)
  H_t = np.zeros((4, 5))

  # Figure out whether yaw is closer to a multiple of pi/2 or pi/4. If
  # it's closer to pi/4, don't trust sensors
  diff = fmodp(state[2], np.pi/2)
  if diff > np.pi/8 and diff < 3*np.pi/8:
    pass
  elif state[2] >= 15*np.pi/8 or state[2] <= np.pi/8:
    # Facing forward
    R[1, 1] = 1000
    R[2, 2] = 1000

    theta = state[2]
    c = np.cos(theta)
    ooc = 1/c
    zhat[3] = state[0]*ooc
    zhat[0] = state[1]*ooc

    H_t[3, 0] = ooc
    H_t[0, 0] = ooc
    ddyaw = sin(theta)/(c*c)
    H_t[3, 2] = state[0]*ddyaw
    H_t[0, 2] = state[1]*ddyaw

  elif state[2] >= 3*np.pi/8 and state[2] <= 5*np.pi/8:
    # Facing left
    R[0, 0] = 1000
    R[1, 1] = 1000
    c = np.cos(state[2] - np.pi/2)

    theta = state[2] - np.pi/2
    c = np.cos(theta)
    ooc = 1/c
    zhat[2] = state[0]*ooc
    zhat[3] = state[1]*ooc

    H_t[2, 0] = ooc
    H_t[3, 1] = ooc
    ddyaw = sin(theta)/(c*c)
    H_t[2, 2] = state[0]*ddyaw
    H_t[3, 2] = state[1]*ddyaw

  elif state[2] >= 7*np.pi/8 and state[2] <= 9*np.pi/8:
    # Facing backward
    R[3, 3] = 1000
    R[0, 0] = 1000
    c = np.cos(state[2] - np.pi)
    zhat[1] = state[0]/c
    zhat[2] = state[1]/c

    theta = state[2] - np.pi
    c = np.cos(theta)
    ooc = 1/c
    zhat[1] = state[0]*ooc
    zhat[2] = state[1]*ooc

    H_t[1, 0] = ooc
    H_t[2, 1] = ooc
    ddyaw = sin(theta)/(c*c)
    H_t[1, 2] = state[0]*ddyaw
    H_t[2, 2] = state[1]*ddyaw

  elif state[2] >= 11*np.pi/8 and state[2] <= 13*np.pi/8:
    # Facing right
    R[2, 2] = 1000
    R[3, 3] = 1000

    theta = state[2] - 3*np.pi/2
    c = np.cos(theta)
    ooc = 1/c
    zhat[0] = state[0]*ooc
    zhat[1] = state[1]*ooc

    H_t[0, 0] = ooc
    H_t[1, 1] = ooc
    ddyaw = sin(theta)/(c*c)
    H_t[0, 2] = state[0]*ddyaw
    H_t[1, 2] = state[1]*ddyaw

  resid = obs - zhat
  inn_cov = np.matmul(np.matmul(H_t, cov), H_t.transpose()) + R
  Ktmp = np.matmul(cov, H_t.transpose())
  state = state + np.matmul(Ktmp, np.linalg.solve(inn_cov, resid))
  cov = np.matmul(np.eye(5) - np.matmul(Ktmp, np.linalg.solve(inn_cov, H_t)), cov)
  return state, cov
