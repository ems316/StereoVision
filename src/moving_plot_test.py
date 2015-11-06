import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

plt.axes()
plt.show(block = False)
for num in range(0,100):
	circle = plt.Circle((num, num), radius=0.1, fc='y')
	print(circle)
	plt.gca().add_patch(circle)
	plt.axis([-10,15,-10,10])
	plt.draw()
	#time.sleep(.2)
	plt.cla()

'''

plt.axes()
plt.show(block = False)
circle = plt.Circle((0, 0), radius=0.75, fc='y')
plt.gca().add_patch(circle)
plt.axis('scaled')
plt.draw()
time.sleep(1)

plt.cla()

points = [[5, 1], [2, 1], [2, 5]]
polygon = plt.Polygon(points)
plt.gca().add_patch(polygon)
plt.axis('scaled')
plt.draw()


time.sleep(1)

plt.cla()

circle = plt.Circle((0, 0), radius=0.45, fc='y')
plt.gca().add_patch(circle)
plt.axis('scaled')
plt.draw()
time.sleep(1)

#line_ani = animation.FuncAnimation(fig1, update_line, 25, fargs=(data, l),
#    interval=50, blit=True)
#line_ani.save('lines.mp4')

#fig2 = plt.figure()

#x = np.arange(-9, 10)
#y = np.arange(-9, 10).reshape(-1, 1)
#base = np.hypot(x, y)
#ims = []
#for add in np.arange(15):
#    ims.append((plt.pcolor(x, y, base + add, norm=plt.Normalize(0, 30)),))

#im_ani = animation.ArtistAnimation(fig2, ims, interval=50, repeat_delay=3000,
#    blit=True)
#im_ani.save('im.mp4', metadata={'artist':'Guido'})
'''
