from re import T
import socket
import sys
from textwrap import fill
import time
import timeit
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np
import math

# Switch matplotlib backend to Qt5Agg
matplotlib.use('Qt5Agg')



HOST = "192.168.1.15" 
PORT = 8001
FREQUENCY = 0.15

robot_position_x = 0
robot_position_z = 0
guide_position = 0
previous_robot_pos_x = 0
previous_robot_pos_z = 0
previous_guide_pos = 0
goaheadflag = 0
start_time = 0
already_go_ahead = False

class TCPServer(socket.socket):

    
    def __init__(self, host, port):
        super().__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port
        self.addr = (self.host, self.port)
        self.stop_requested = False

    def setup_connection(self):
        self.bind(self.addr)
        self.listen(1)

    def start_reading(self):
        self.readingThread = threading.Thread(target=self.read_from_client)
        self.readingThread.start()

    def read_from_client(self):
        while True and not self.stop_requested:
            print('Waiting for a connection')
            connection, client_address = self.accept()
            try:
                print('connection from', client_address)
                while True and not self.stop_requested:
                    data = connection.recv(30)
                    if data:
                        self.parse_data(data)
            finally:
                connection.close()

    def parse_data(self, data:bytes):
        global robot_position_x
        global robot_position_z
        global guide_position
        global goaheadflag
        global previous_robot_pos_x
        global previous_robot_pos_z
        global previous_guide_pos
        global start_time
        global already_go_ahead
        # print("byte: ", data)
        data = data.decode().split(";")
        # print("decoded: ", data)
        # if  not already_go_ahead and len(data) == 1:
        #     goaheadflag = 1
        #     already_go_ahead = True
            # start_time = timeit.default_timer()
        #else:
        try:
            robot_position_x = float(data[1])
            robot_position_z = float(data[2])
            guide_position = float(data[3])
            previous_robot_pos_x = robot_position_x
            previous_robot_pos_z = robot_position_z
            previous_guide_pos = guide_position
            # print(robot_position ,  " " , str(guide_position))      
        except Exception as e:
            print(e)
            print("data was: ")
            print(data)
            robot_position_x = previous_robot_pos_x
            robot_position_z = previous_robot_pos_z
            guide_position = previous_guide_pos 
        

    def close(self):
        self.stop_requested = True
        self.readingThread.join()


class Drawer():

    def __init__(self, guide_direction, diameter_1=False , diameter_2=False , circular_cw=False, circular_ccw=False):
        self.dir = guide_direction
        self.dia_1 = diameter_1
        self.dia_2 = diameter_2
        self.cir_cw = circular_cw
        self.cir_ccw = circular_ccw
        self.circular_motions_radius = 10 * math.sqrt(2);              
        pass

    def start_animation(self):
        self.fig, self.ax = plt.subplots(ncols=1, nrows=1)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=1, init_func=self.initilize_plot)
        self.fig.canvas.manager.set_window_title('Live Robot Position')
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()


    def update_plot(self, frame):

        t = time.time() - self.start_time
        # self.periodic_box.set_width(self.bounding_box.get_width() / 2 * np.sin(2 * np.pi * t * FREQUENCY))
        if self.dia_1 == True:
            self.guide_circle.set_center((25 + 10*math.sqrt(2)*(guide_position/20), 22.5 + 10*math.sqrt(2)*(guide_position/20)))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z)) 

        elif self.dia_2 == True:
            self.guide_circle.set_center((25 + 10*math.sqrt(2)*(guide_position/20), 22.5 - 10*math.sqrt(2)*(guide_position/20)))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))

        elif self.cir_cw == True:
            self.guide_circle.set_center((25 + self.circular_motions_radius*np.cos(2 * np.pi * t * FREQUENCY), 22.5 + self.circular_motions_radius*np.sin(2 * np.pi * t * FREQUENCY)))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))     

        elif self.cir_ccw == True:
            self.guide_circle.set_center((25 + self.circular_motions_radius*np.cos(-2 * np.pi * t * FREQUENCY), 22.5 + self.circular_motions_radius*np.sin(-2 * np.pi * t * FREQUENCY)))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z)) 

        elif self.dir == 'horizontal':
            self.periodic_box.set_width(guide_position)
            self.guide_line.set_xdata([25 + guide_position, 25 + guide_position])
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z)) 

        elif self.dir == 'vertical':
            self.periodic_box.set_height(guide_position)
            self.guide_line.set_ydata([22.5 + guide_position, 22.5 + guide_position])
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))            
        
    def initilize_plot(self):
        self.start_time = time.time()
        self.ax.set_aspect('equal')
        self.ax.set_title('Live Robot Position', fontsize=20, fontweight='bold')
        self.ax.grid(False, which='both') 
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        self.ax.set_xlim(0, 50)
        self.ax.set_ylim(0, 50)
        self.ax.tick_params(
            axis='both', which='both', bottom=False, top=False, labelbottom=False, right=False, left=False, labelleft=False)

        if self.dia_1 == True:
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.diagonal_line_1 = plt.Line2D([25-10*math.sqrt(2),25+10*math.sqrt(2)],[22.5-10*math.sqrt(2),22.5+10*math.sqrt(2)], color='k', linestyle='--')
            self.ax.add_line(self.diagonal_line_1)
            self.guide_circle = plt.Circle((25,22.5), 3, color='k', fill=False)
            self.ax.add_patch(self.guide_circle)

        elif self.dia_2 == True:
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.diagonal_line_2 = plt.Line2D([25-10*math.sqrt(2),25+10*math.sqrt(2)],[22.5+10*math.sqrt(2),22.5-10*math.sqrt(2)], color='k', linestyle='--')
            self.ax.add_line(self.diagonal_line_2)
            self.guide_circle = plt.Circle((25,22.5), 3, color='k', fill=False)
            self.ax.add_patch(self.guide_circle)

        elif self.cir_cw == True:
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            # self.path_circle = plt.Circle((25,22.5), 15, color='k', fill=False, linestyle='--')
            self.path_circle = plt.Circle((25,22.5), self.circular_motions_radius, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.path_circle)
            # self.guide_circle = plt.Circle((25+15,22.5), 3, color='k', fill=False)
            self.guide_circle = plt.Circle((25+self.circular_motions_radius,22.5), 3, color='k', fill=False)
            self.ax.add_patch(self.guide_circle)

        elif self.cir_ccw == True:
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            # self.path_circle = plt.Circle((25,22.5), 15, color='k', fill=False, linestyle='--')
            self.path_circle = plt.Circle((25,22.5), self.circular_motions_radius, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.path_circle)
            # self.guide_circle = plt.Circle((25+15,22.5), 3, color='k', fill=False)
            self.guide_circle = plt.Circle((25+self.circular_motions_radius,22.5), 3, color='k', fill=False)
            self.ax.add_patch(self.guide_circle)

        elif self.dir == 'horizontal':    
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.periodic_box = plt.Rectangle((25, 21.25), 0, 2.5, fc='r')
            self.guide_line = plt.Line2D([25,25], [19,26], color='r', lw=2)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            # self.guide_circle = plt.Circle((25,22.5), 15, color='k', fill=False, linestyle='--')
            self.path_circle = plt.Circle((25,22.5), self.circular_motions_radius, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.periodic_box)
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.guide_line)
            # self.ax.add_patch(self.guide_circle)

        elif self.dir == 'vertical':
            self.horizontal_line = plt.Line2D([5,45],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = self.horizontal_line.get_xdata()[1] - self.horizontal_line.get_xdata()[0]
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)
            self.periodic_box = plt.Rectangle((23.75, 22.5), 2.5, 0, fc='r')
            self.guide_line = plt.Line2D([21.5,28.5], [22.5,22.5], color='r', lw=2)
            self.cursor_circle = plt.Circle((22.5,25), 2, color='b')
            # self.guide_circle = plt.Circle((25,22.5), 15, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.periodic_box)
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.guide_line)
            # self.ax.add_patch(self.guide_circle)

    def close(self):
        plt.close()

if __name__ == '__main__':
    server = TCPServer(HOST, PORT)
    server.setup_connection()
    server.start_reading()
    drawer = Drawer(guide_direction='horizontal')
    drawer.start_animation()
    drawer.close()
    server.close()
    sys.exit(0)