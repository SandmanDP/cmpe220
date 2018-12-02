import json
from PIL import Image, ImageTk
import serial
import threading
import time
import tkinter as tk


big_map_coords = {
    'top_left': (42.257923, -126.708527),
    # 'top_right': (42.257923, -112.818561),
    # 'bottom_left': (32.105504, -126.708527),
    'bottom_right': (32.105504, -112.818561),
    'dimensions': (1000, 920)
}


small_map_coords = {
    'top_left': (37.339309, -121.887050),
    'bottom_right': (37.330863, -121.875505),
    'dimensions': (1000, 920)
}


def get_xy_pos(map_dict, coord_pair):
    return (int((coord_pair[1]-map_dict['top_left'][1])/(map_dict['bottom_right'][1]-map_dict['top_left'][1])*map_dict['dimensions'][0]),
            int((coord_pair[0]-map_dict['top_left'][0])/(map_dict['bottom_right'][0]-map_dict['top_left'][0])*map_dict['dimensions'][1]))


cold_temp = 60
hot_temp = 210
def temp_to_color(temp):
    red = 0
    green = 0
    blue = 0
    if temp <= cold_temp:
        blue = 255
    elif temp >= hot_temp:
        red = 255
    elif cold_temp <= temp <= cold_temp + 50:
        green = int((temp-cold_temp)/50*255)
        blue = int((cold_temp+50-temp)/50*255)
    elif cold_temp + 50 <= temp <= hot_temp - 50:
        red = int((temp-50-cold_temp)/50*255)
        green = 255
    else:
        red = 255
        green = int((hot_temp-temp)/50*255)
    return f'#{red:02x}{green:02x}{blue:02x}'


class Fireman:

    def __init__(self):
        self.position = (None, None)
        self.direction = None
        self.temperature = None
        self.air_quality = None
        self.hazardous_temp = 0
        self.hazardous_air = 0
        self.active = False


class FireApp(tk.Tk):

    def __init__(self):
        self.window = tk.Tk.__init__(self)
        self.canvas = tk.Canvas(self.window, width=1000, height=920, highlightthickness=0, relief='ridge')
        self.canvas.pack_propagate(0)
        self.canvas.grid()
        self.title('Firefighter Monitor')
        self.bigimage = ImageTk.PhotoImage(Image.open('bigmap.png'))
        self.smallimage = ImageTk.PhotoImage(Image.open('smallmap.png'))
        self.image = self.canvas.create_image(0, 0, anchor='nw', image=self.bigimage)
        # self.imglabel = tk.Label(self.canvas, image=self.bigimage)
        # self.imglabel.image = self.bigimage
        # self.imglabel.pack()
        # self.imglabel.place()
        self.is_big = True
        # self.next_image = self.smallimage
        self.bind('<Return>', self.enter)
        self.protocol("WM_DELETE_WINDOW", self.ender)
        # self.bind('<Control-Shift-space>', self.space_pressed)
        # self.protocol("WM_DELETE_WINDOW", self.on_delete)
        self.widgets = []
        self.firemen = [Fireman()]
        self.firemen_markers = []
        self.json_listener = SerialTask(self)
        self.json_listener.start()
        # self.update_firefighters()
        self.mainloop()

    def enter(self, *args):
        # self.imglabel.destroy()
        # self.bigimglabel.pack_forget()
        # self.imglabel.image = self.smallimage
        # self.smallimglabel.pack()
        # self.smallimglabel.place()
        # self.imglabel.configure(image=self.next_image)
        # self.imglabel.image = self.next_image
        self.canvas.itemconfigure(self.image, image=self.smallimage if self.is_big else self.bigimage)
        self.is_big = not self.is_big
        self.update_firefighters()
        # print("Ding!")

    def ender(self):
        self.json_listener.running = False
        self.destroy()

    def update_firefighters(self):
        for item in self.firemen_markers:
            self.canvas.delete(item)
        self.firemen_markers = []
        for fireman in self.firemen:
            if fireman.active:
                # self.canvas.mo
                fireman_x, fireman_y = get_xy_pos(big_map_coords if self.is_big else small_map_coords, fireman.position)
                self.firemen_markers.append(self.canvas.create_oval(fireman_x-15, fireman_y-15, fireman_x+15, fireman_y+15, fill=temp_to_color(fireman.temperature)))
                self.firemen_markers.append(self.canvas.create_arc(fireman_x-15, fireman_y-15, fireman_x+15, fireman_y+15, extent=60, start=(90-fireman.direction+360-30)%360, fill='white'))


class SerialTask(threading.Thread):

    def __init__(self, tkwin):
        threading.Thread.__init__(self)
        self.tkwin = tkwin
        self.running = True

    def run(self):
        with serial.Serial(port='/dev/cu.usbserial-A503JOHS', baudrate=38400, timeout=1) as hub:
            # hub.open()
            jstr = '{"temperature": 62, "direction": 270, "gps_lat": 37.336, "gps_long": -121.88}'
            while(self.running):
                print(hub.readline())
                jdict = json.loads(jstr)
                self.tkwin.firemen[0].position = (jdict['gps_lat'], jdict['gps_long'])
                self.tkwin.firemen[0].temperature = jdict['temperature']
                self.tkwin.firemen[0].direction = jdict['direction']
                self.tkwin.firemen[0].active = True
                self.tkwin.update_firefighters()
                # time.sleep(1)


FireApp()