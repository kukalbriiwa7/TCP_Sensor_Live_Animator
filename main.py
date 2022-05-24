import socket
import sys
import time
import timeit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np

HOST = "192.168.1.15" 
PORT = 8001
FREQUENCY = 0.25

robot_position = 0
guide_position = 0
previous_robot_pos = 0
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
        global robot_position
        global guide_position
        global goaheadflag
        global previous_robot_pos
        global previous_guide_pos
        global start_time
        global already_go_ahead
        print("byte: ", data)
        data = data.decode().split(";")
        # print("decoded: ", data)
        # if  not already_go_ahead and len(data) == 1:
        #     goaheadflag = 1
        #     already_go_ahead = True
            # start_time = timeit.default_timer()
        #else:
        try:
            robot_position = float(data[1])
            guide_position = float(data[2])
            previous_robot_pos = robot_position
            previous_guide_pos = guide_position
            # print(robot_position ,  " " , str(guide_position))      
        except Exception as e:
            print(e)
            print("data was: ")
            print(data)
            robot_position = previous_robot_pos
            guide_position = previous_guide_pos 
        

    def close(self):
        self.stop_requested = True
        self.readingThread.join()


class Drawer():
    def __init__(self):
        pass

    def start_animation(self):
        self.fig, self.ax = plt.subplots(ncols=1, nrows=1)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=1, init_func=self.initilize_plot)
        self.fig.canvas.manager.set_window_title('Live Robot Position')
        plt.show()

    def update_plot(self, frame):
        # t = time.time() - self.start_time
        # self.periodic_box.set_width(self.bounding_box.get_width() / 2 * np.sin(2 * np.pi * t * FREQUENCY))
        self.periodic_box.set_width(guide_position)
        self.cursor_line.set_xdata([25 + robot_position, 25 + robot_position])
        #print("Hello")
        # if not already_go_ahead:
        #     if goaheadflag == 1:
        #         self.ax.text(25,10,'Go Ahead',fontsize=24,horizontalalignment='center')
        #         TCPServer.goaheadflag = 0
        #         TCPServer.already_go_ahead = True
           
        
    def initilize_plot(self):
        self.start_time = time.time()
        self.ax.set_title('Live Robot Position')
        self.ax.grid(False, which='both') 
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        self.ax.set_xlim(0, 50)
        self.ax.set_ylim(0, 50)
        self.ax.tick_params(
            axis='both', which='both', bottom=False, top=False, labelbottom=False, right=False, left=False, labelleft=False)

        self.bounding_box = plt.Rectangle((5, 24), 40, 5, ec='y', fc='none')
        self.periodic_box = plt.Rectangle((25, 24), 0, 5, fc='r')
        self.cursor_line = plt.Line2D([25,25], [24,29], color='k', lw=2)
        self.ax.add_patch(self.bounding_box)
        self.ax.add_patch(self.periodic_box)
        self.ax.add_line(self.cursor_line)

    def close(self):
        plt.close()


if __name__ == '__main__':
    server = TCPServer(HOST, PORT)
    server.setup_connection()
    server.start_reading()
    drawer = Drawer()
    drawer.start_animation()
    drawer.close()
    server.close()
    sys.exit(0)