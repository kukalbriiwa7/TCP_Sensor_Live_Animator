import socket
import sys
import time
import timeit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np

fig, ax = plt.subplots(ncols=1, nrows=1)

ax.grid(False, which='both') 
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.tick_params(
    axis='both', which='both', bottom=False, top=False, labelbottom=False, right=False, left=False, labelleft=False)

bounding_box = plt.Rectangle((5, 20), 40, 5, ec='y', fc='none')
horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k')
# vertical_line = plt.Line2D([25,25],[horizontal_line.get_xdata(orig = True),35], color='k')
periodic_box = plt.Rectangle((25, 20), 0, 5, fc='r')
cursor_line = plt.Line2D([25,25], [20,25], color='k', lw=2)
cursor_circle = plt.Circle((25,20), 0.5, color='r')

ax.add_patch(bounding_box)
ax.add_patch(periodic_box)
ax.add_line(cursor_line)
ax.add_line(horizontal_line)
ax.add_patch(cursor_circle)
# ax.add_line(vertical_line)


# plt.show()

print(cursor_circle.get_center()[0])