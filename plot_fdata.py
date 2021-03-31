import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

fig = plt.figure()
plt.plot([152.4, 0, 0], [0, 0, 91.44])
quiv = plt.quiver([0], [0], [0], [0])
#filter_data = np.load('1617198122.npy')

def animate(i): 
  global quiv
  quiv.remove()
  quiv = plt.quiver(filter_data[0, :, i], filter_data[1, :, i], np.cos(filter_data[2, :, i] - np.pi/2), np.sin(filter_data[2, :, i] - np.pi/2))
  return quiv, 
	
# call the animator	 
anim = animation.FuncAnimation(fig, animate, frames=filter_data.shape[2], interval=100, blit=True, repeat=False)

plt.show()
