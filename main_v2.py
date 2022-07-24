from re import T
import socket
import sys
from textwrap import fill
import time
import timeit
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import matplotlib.animation as animation
import threading
import numpy as np
import math

# Switch matplotlib backend to Qt5Agg
matplotlib.use('Qt5Agg')

HOST = "192.168.1.15" 
PORT = 8001
FREQUENCY = 0.0625

robot_position_x = 0
robot_position_z = 0
robot_eul_1 = 0
previous_robot_pos_x = 0
previous_robot_pos_z = 0
previous_eul_1 = 0

guide_position_x = 0
guide_position_z = 0
guide_eul_angle = 0
previous_guide_position_x = 0
previous_guide_position_z = 0
previous_guide_eul_angle = 0

goaheadflag = 0
start_time = 0
already_go_ahead = False
motion_started = False

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
                    data = connection.recv(60)
                    if data:
                        self.parse_data(data)
            finally:
                connection.close()

    def parse_data(self, data:bytes):
        global robot_position_x
        global robot_position_z
        global robot_eul_1
        global previous_robot_pos_x
        global previous_robot_pos_z
        global previous_eul_1

        global guide_position_x
        global guide_position_z
        global guide_eul_angle
        global previous_guide_position_x
        global previous_guide_position_z
        global previous_guide_eul_angle

        global goaheadflag

        global start_time
        global already_go_ahead

        # print("byte: ", data)
        data = data.decode().split(";")
        print("decoded: ", data)
        # if  not already_go_ahead and len(data) == 1:
        #     goaheadflag = 1
        #     already_go_ahead = True
            # start_time = timeit.default_timer()
        #else:
        try:
            robot_position_x = float(data[1])
            robot_position_z = float(data[2])
            robot_eul_1 = -float(data[3])
            guide_position_x = float(data[4])
            guide_position_z = float(data[5])
            guide_eul_angle = float(data[6])

            previous_robot_pos_x = robot_position_x
            previous_robot_pos_z = robot_position_z
            previous_eul_1 = robot_eul_1
            previous_guide_position_x = guide_position_x
            previous_guide_position_z = guide_position_z
            previous_guide_eul_angle = guide_eul_angle
            # print(robot_position ,  " " , str(guide_position))      
        except Exception as e:
            print(e)
            print("data was: ")
            print(data)

            robot_position_x = previous_robot_pos_x
            robot_position_z = previous_robot_pos_z
            robot_eul_1 = previous_eul_1
            guide_position_x = previous_guide_position_x
            guide_position_z = previous_guide_position_z
            guide_eul_angle = previous_guide_eul_angle

    def close(self):
        self.stop_requested = True
        self.readingThread.join()


class Drawer():
    def __init__(self, des_motion_amplitude, u_maneuver_radius = None ,horizonal = False, vertical = False, diameter_1=False , diameter_2=False , circular_cw=False, circular_ccw=False, pure_rotation=False, u_shaped_maneuver=False, lemniscate_shaped_maneuver = False):
        
        if u_maneuver_radius:
            self.radius = u_maneuver_radius
        self.hor = horizonal
        self.ver = vertical
        self.dia_1 = diameter_1
        self.dia_2 = diameter_2
        self.cir_cw = circular_cw
        self.cir_ccw = circular_ccw
        self.pure_rot = pure_rotation
        self.u_maneuver = u_shaped_maneuver
        self.lemniscate_maneuver = lemniscate_shaped_maneuver
        self.circular_motions_radius = des_motion_amplitude * math.sqrt(2)/2
        self.r = 5
        self.lemniscate_max_radius = 10
        self.lemniscate_c = math.sqrt((self.lemniscate_max_radius)**2/2)
        self.previous_lemniscate_x = self.lemniscate_max_radius
        self.previous_lemniscate_y = 0 
        self._amplitude = des_motion_amplitude        
        

    def compute_local_normal_angle(self, x, y, x_prev, y_prev):
        c = self.lemniscate_c
        if x == 0 and y == 0:
            x = x_prev
            y = y_prev
        local_tangent_slope = -(4*x**3+4*x*y**2-4*c**2*x)/(4*y**3+4*x**2*y+4*c**2*y)
        local_normal_slope = -1/local_tangent_slope
        lemniscate_critical_point = self.lemniscate_max_radius*np.sqrt(6)/4
        if x > lemniscate_critical_point or x < -lemniscate_critical_point:
            theta = np.pi/2
        else:
            theta = np.arctan(local_normal_slope)
        return theta


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
            self.guide_circle.set_center((25 + guide_position_x, 22.5 + guide_position_z))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z)) 

        elif self.dia_2 == True:
            self.guide_circle.set_center((25 + guide_position_x, 22.5 + guide_position_z))
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))

        elif self.cir_cw == True:
            self.guide_circle.set_center((25 + guide_position_x, 22.5 + guide_position_z))
            self.guide_handle.set_xdata([25 + guide_position_x - self.r*np.cos(guide_eul_angle), 25 + guide_position_x + self.r*np.cos(guide_eul_angle)])
            self.guide_handle.set_ydata([22.5 + guide_position_z - self.r*np.sin(guide_eul_angle), 22.5 + guide_position_z + self.r*np.sin(guide_eul_angle)])
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))
            self.cursor_handle.set_xdata([25 + robot_position_x - self.r*np.sin(robot_eul_1), 25 + robot_position_x + self.r*np.sin(robot_eul_1)])
            self.cursor_handle.set_ydata([22.5 + robot_position_z - self.r*np.cos(robot_eul_1), 22.5 + robot_position_z + self.r*np.cos(robot_eul_1)]) 

        elif self.cir_ccw == True:
            self.guide_circle.set_center((25 + guide_position_x, 22.5 + guide_position_z))
            self.guide_handle.set_xdata([25 + guide_position_x - self.r*np.cos(guide_eul_angle), 25 + guide_position_x + self.r*np.cos(guide_eul_angle)])
            self.guide_handle.set_ydata([22.5 + guide_position_z - self.r*np.sin(guide_eul_angle), 22.5 + guide_position_z + self.r*np.sin(guide_eul_angle)])
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))
            self.cursor_handle.set_xdata([25 + robot_position_x - self.r*np.sin(robot_eul_1), 25 + robot_position_x + self.r*np.sin(robot_eul_1)])
            self.cursor_handle.set_ydata([22.5 + robot_position_z - self.r*np.cos(robot_eul_1), 22.5 + robot_position_z + self.r*np.cos(robot_eul_1)]) 

        elif self.pure_rot == True:
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))
            self.guide_handle.set_xdata([25 - 12*np.sin(guide_eul_angle), 25 + 12*np.sin(guide_eul_angle)])
            self.guide_handle.set_ydata([22.5 - 12*np.cos(guide_eul_angle), 22.5 + 12*np.cos(guide_eul_angle)])
            self.cursor_handle.set_xdata([25 + robot_position_x - 10*np.sin(robot_eul_1), 25 + robot_position_x + 10*np.sin(robot_eul_1)])
            self.cursor_handle.set_ydata([22.5 + robot_position_z - 10*np.cos(robot_eul_1), 22.5 + robot_position_z + 10*np.cos(robot_eul_1)])

        elif self.u_maneuver == True:
            # theta = 2*np.pi/6*np.sin(2*np.pi*FREQUENCY*(t-1/(4*FREQUENCY))) + np.pi/2
            x_c = self.radius*np.cos(-guide_eul_angle)
            y_c = self.radius/2 + self.radius*np.sin(-guide_eul_angle)
            # y_c = self.circular_motions_radius*np.sin(-theta)
            self.guide_circle.set_center((25 + x_c, 22.5 + y_c))
            self.guide_handle.set_xdata([25 + x_c - self.r*np.cos(-guide_eul_angle), 25 + x_c + self.r*np.cos(-guide_eul_angle)])
            self.guide_handle.set_ydata([22.5 + y_c - self.r*np.sin(-guide_eul_angle), 22.5 + y_c + self.r*np.sin(-guide_eul_angle)])

            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))    
            self.cursor_handle.set_xdata([25 + robot_position_x - self.r*np.sin(robot_eul_1), 25 + robot_position_x + self.r*np.sin(robot_eul_1)])
            self.cursor_handle.set_ydata([22.5 + robot_position_z - self.r*np.cos(robot_eul_1), 22.5 + robot_position_z + self.r*np.cos(robot_eul_1)])

        elif self.lemniscate_maneuver == True: 

            self.guide_circle.set_center((25 + guide_position_x, 22.5 + guide_position_z))
            self.guide_handle.set_xdata([25 + guide_position_x - self.r*np.cos(guide_eul_angle), 25 + guide_position_x + self.r*np.cos(guide_eul_angle)])
            self.guide_handle.set_ydata([22.5 + guide_position_z - self.r*np.sin(guide_eul_angle), 22.5 + guide_position_z + self.r*np.sin(guide_eul_angle)])

            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z))    
            self.cursor_handle.set_xdata([25 + robot_position_x - self.r*np.sin(robot_eul_1), 25 + robot_position_x + self.r*np.sin(robot_eul_1)])
            self.cursor_handle.set_ydata([22.5 + robot_position_z - self.r*np.cos(robot_eul_1), 22.5 + robot_position_z + self.r*np.cos(robot_eul_1)])

        elif self.hor == True:
            self.periodic_box.set_width(guide_position_x)
            self.guide_line.set_xdata([25 + guide_position_x, 25 + guide_position_x])
            self.cursor_circle.set_center((25 + robot_position_x, 22.5 + robot_position_z)) 

        elif self.ver == True:
            self.periodic_box.set_height(guide_position_z)
            self.guide_line.set_ydata([22.5 + guide_position_z, 22.5 + guide_position_z])
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
            self._amplitude = self._amplitude * 100
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.diagonal_line_1 = plt.Line2D([25-self._amplitude*math.sqrt(2)/2,25+self._amplitude*math.sqrt(2)/2],[22.5-self._amplitude*math.sqrt(2)/2,22.5+self._amplitude*math.sqrt(2)/2], color='k', linestyle='--')
            self.guide_circle = plt.Circle((25,22.5), 3, color='k', fill=False)

            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.diagonal_line_1)
            self.ax.add_patch(self.guide_circle)

        elif self.dia_2 == True:
            self._amplitude = self._amplitude * 100
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')
            self.diagonal_line_2 = plt.Line2D([25-self._amplitude*math.sqrt(2)/2,25+self._amplitude*math.sqrt(2)/2],[22.5+self._amplitude*math.sqrt(2)/2,22.5-self._amplitude*math.sqrt(2)/2], color='k', linestyle='--')
            self.guide_circle = plt.Circle((25,22.5), 3, color='k', fill=False)

            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.diagonal_line_2)
            self.ax.add_patch(self.guide_circle)

        elif self.cir_cw == True:
            self._amplitude = self._amplitude * 100
            self.circular_motions_radius = self.circular_motions_radius * 100
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.cursor_circle = plt.Circle((25,22.5), 1, color='r')
            self.cursor_handle = plt.Line2D([25,25],[27.5,17.5], color='r', linewidth=1.5)

            self.guide_circle = plt.Circle((25+self.circular_motions_radius,22.5), 3, color='k', fill=False)
            self.guide_handle = plt.Line2D([25+self.circular_motions_radius,25+self.circular_motions_radius],[22.5+self.r,22.5-self.r], color='k', linewidth=1)

            self.path_circle = plt.Circle((25,22.5), self.circular_motions_radius, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.path_circle)

            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.cursor_handle)
            self.ax.add_patch(self.guide_circle)
            self.ax.add_line(self.guide_handle)

        elif self.cir_ccw == True:
            self._amplitude = self._amplitude * 100
            self.circular_motions_radius = self.circular_motions_radius * 100


            self.cursor_circle = plt.Circle((25,22.5), 1, color='r')
            self.cursor_handle = plt.Line2D([25,25],[27.5,17.5], color='r', linewidth=1.5)

            self.guide_circle = plt.Circle((25+self.circular_motions_radius,22.5), 3, color='k', fill=False)
            self.guide_handle = plt.Line2D([25+self.circular_motions_radius,25+self.circular_motions_radius],[22.5+self.r,22.5-self.r], color='k', linewidth=1)

            self.path_circle = plt.Circle((25,22.5), self.circular_motions_radius, color='k', fill=False, linestyle='--')
            self.ax.add_patch(self.path_circle)
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.cursor_handle)
            self.ax.add_patch(self.guide_circle)
            self.ax.add_line(self.guide_handle)

        elif self.u_maneuver == True:
            self.radius = self.radius * 100
            axis_length = self.radius * 2/np.sqrt(2)

            self.horizontal_line = plt.Line2D([25-axis_length,25+axis_length],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*axis_length
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.half_circle = Arc((25,22.5+self.radius/2), 2*self.radius, 2*self.radius, theta1 = np.rad2deg(3*np.pi/2 - self._amplitude) ,
                                     theta2=-np.rad2deg(np.pi/2 - self._amplitude), color='k', fill=False)

            self.guide_circle = plt.Circle((25+self.radius*np.cos(-(np.pi/2 - self._amplitude)),
                                            22.5 + self.radius/2 + self.radius*np.sin(-(np.pi/2 - self._amplitude))), 1, color='b', fill=True)

            self.guide_handle = plt.Line2D([25+self.radius*np.cos(-(np.pi/2 - self._amplitude))-self.r*np.cos(-(np.pi/2 - self._amplitude)),
                                            25+self.radius*np.cos(-(np.pi/2 - self._amplitude))+self.r*np.cos(-(np.pi/2 - self._amplitude))]
                                            ,[22.5+self.radius/2+self.radius*np.sin(-(np.pi/2 - self._amplitude))-self.r*np.sin(-(np.pi/2 - self._amplitude)),
                                              22.5+self.radius/2+self.radius*np.sin(-(np.pi/2 - self._amplitude))+self.r*np.sin(-(np.pi/2 - self._amplitude))],
                                            color='b', linewidth=2)

            self.cursor_handle = plt.Line2D([25,25],[27.5,17.5], color='r', linewidth=1.5)
            self.cursor_circle = plt.Circle((25,22.5), 1, color='r')

            self.horizontal_line_center = plt.Line2D([22.5,27.5],[22.5+self.radius/2,22.5+self.radius/2], color='k', linewidth=4)
            horizonal_line__center_length = self.horizontal_line_center.get_xdata()[1] - self.horizontal_line_center.get_xdata()[0]
            self.vertical_line_center = plt.Line2D([25,25],[self.horizontal_line_center.get_ydata(orig = True)[0]+horizonal_line__center_length/2,
                                            self.horizontal_line_center.get_ydata(orig = True)[0]-horizonal_line__center_length/2], color='k', linewidth=4)

            
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.half_circle)
            self.ax.add_patch(self.guide_circle)
            self.ax.add_line(self.guide_handle)
            self.ax.add_line(self.cursor_handle)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.horizontal_line_center)
            self.ax.add_line(self.vertical_line_center)

        elif self.lemniscate_maneuver == True:

            self._amplitude = self._amplitude * 100
            self.lemniscate_c = math.sqrt((self._amplitude)**2/2)

            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                                     self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.guide_circle = plt.Circle((25+self._amplitude,22.5), 1, color='b', fill=True)
            self.guide_handle = plt.Line2D([25+self._amplitude,25+self._amplitude],[22.5+self.r,22.5-self.r], color='b', linewidth=2)

            self.cursor_handle = plt.Line2D([25,25],[27.5,17.5], color='r', linewidth=1.5)
            self.cursor_circle = plt.Circle((25,22.5), 1, color='r')     

            delta = 0.025
            xrange = np.arange(0, 50, delta)
            yrange = np.arange(0, 50, delta)
            x, y = np.meshgrid(xrange, yrange)
            equation = ((x-25)**2 + (y-22.5)**2)**2 -2*self.lemniscate_c**2*((x-25)**2 - (y-22.5)**2)
            plt.contour(x, y, equation, [0])
            plt.show()

            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.guide_circle)
            self.ax.add_line(self.guide_handle)
            self.ax.add_line(self.cursor_handle)
            self.ax.add_patch(self.cursor_circle)    

        elif self.pure_rot == True:

            self.horizontal_line = plt.Line2D([25-20,25+20],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*20
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.cursor_handle = plt.Line2D([25,25],[32.5,12.5], color='r', linewidth=3)
            self.cursor_circle = plt.Circle((25,22.5), 1.5, color='r')


            self.guide_handle = plt.Line2D([25,25],[34.5,10.5], color='b', linewidth=3)
            self.guide_circle = plt.Circle((25,22.5), 2, color='b')


            self.boundary_line_right = plt.Line2D([25,25+15*np.cos(np.pi/2-self._amplitude)],[22.5+0,22.5+15*np.sin(np.pi/2-self._amplitude)],color='c', linewidth=1)
            self.boundary_line_left = plt.Line2D([25,25-15*np.cos(np.pi/2-self._amplitude)],[22.5+0,22.5+15*np.sin(np.pi/2-self._amplitude)],color='c', linewidth=1)


            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)  
            self.ax.add_patch(self.guide_circle)
            self.ax.add_line(self.guide_handle)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.cursor_handle)
            self.ax.add_line(self.boundary_line_left)     
            self.ax.add_line(self.boundary_line_right)      

        elif self.hor == True:
            self._amplitude = self._amplitude * 100   
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)


            self.periodic_box = plt.Rectangle((25, 21.25), 0, 2.5, fc='r')
            self.guide_line = plt.Line2D([25,25], [19,26], color='r', lw=2)
            self.cursor_circle = plt.Circle((25,22.5), 2, color='b')


            self.ax.add_patch(self.periodic_box)
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.guide_line)

        elif self.ver == True:
            self._amplitude = self._amplitude * 100
            self.horizontal_line = plt.Line2D([25-self._amplitude,25+self._amplitude],[22.5,22.5], color='k', linewidth=3)
            horizonal_line_length = 2*self._amplitude
            self.vertical_line = plt.Line2D([25,25],[self.horizontal_line.get_ydata(orig = True)[0]+horizonal_line_length/2,
                                            self.horizontal_line.get_ydata(orig = True)[0]-horizonal_line_length/2], color='k', linewidth=3)

            self.periodic_box = plt.Rectangle((23.75, 22.5), 2.5, 0, fc='r')
            self.guide_line = plt.Line2D([21.5,28.5], [22.5,22.5], color='r', lw=2)
            self.cursor_circle = plt.Circle((22.5,25), 2, color='b')

            self.ax.add_patch(self.periodic_box)
            self.ax.add_line(self.horizontal_line)
            self.ax.add_line(self.vertical_line)
            self.ax.add_patch(self.cursor_circle)
            self.ax.add_line(self.guide_line)

    def close(self):
        plt.close()

if __name__ == '__main__':
    server = TCPServer(HOST, PORT)
    server.setup_connection()
    server.start_reading()
    drawer = Drawer(lemniscate_shaped_maneuver=True, des_motion_amplitude=0.2)
    drawer.start_animation()
    drawer.close()
    server.close()
    sys.exit(0)