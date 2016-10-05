import sys
import time
from argparse import ArgumentParser
from ConfigParser import SafeConfigParser

from visual import box, sphere, cylinder, vector, color, rate, ring

import PyDragonfly
from PyDragonfly import Dragonfly_Module, MT_EXIT, CMessage, copy_to_msg, copy_from_msg
from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc

import numpy as np
import time

class RTFT(object):
    def __init__(self, config_file, server):
        self.load_config(config_file)
        self.init_gui()
        self.setup_dragonfly(server)
        self.solo = True #false if executed from demigod executive file
        self.RTFT_display = True #default = True. if message from executive, then use that value
        self.state = -1 #-1= between trials 0 = outside target, 1 = close enough, waiting, 2 = close enough, hold time met
        self.start_hold = time.time()
        self.run()
        
    def load_config(self, config_file): #Default config file is RTFT_CONFIG
        self.config = SafeConfigParser()
        self.config.read(config_file)
        self.rate = float(self.config.get('main', 'rate'))
        self.target_vector = [float(x) for x in self.config.get('main', 'target_vector').split(" ")] 
        self.target_color=[float(x) for x in self.config.get('main', 'target_color').split(" ")]
        self.target_rad = float(self.config.get('main', 'target_radius'))
        self.ball_rad = float(self.config.get('main', 'cursor_radius'))
        self.ball_color = [float(x) for x in self.config.get('main', 'cursor_color').split(" ")]
        self.max_factor = float(self.config.get('main', 'max_factor'))
        self.force_scale = float(self.config.get('main', 'force_scale'))
        self.threshold = float(self.config.get('main', 'threshold'))
        self.hold_time = float(self.config.get('main', 'hold_time'))       
    
    def setup_dragonfly(self, server):
        subscriptions = [MT_EXIT, \
                         rc.MT_PING, \
                         rc.MT_FT_DATA, \
                         rc.MT_FT_COMPLETE, \
                         rc.MT_RTFT_CONFIG]
        self.mod = Dragonfly_Module(0, 0)
        self.mod.ConnectToMMM(server)
        for sub in subscriptions:
            self.mod.Subscribe(sub)
        self.mod.SendModuleReady()
        print "Connected to Dragonfly at ", server
        
    def init_gui(self): 
           # Is the orientation matrix missing here????
        self.length   = 100    
        wallR    = box(pos=vector(self.length/2.,0,0), size=(0.2,self.length,self.length), color=color.green)
        wallB    = box(pos=vector(0,0,-self.length/2.), size=(self.length,self.length,0.2), color=color.white)
        wallDown = box(pos=vector(0,-self.length/2.,0), size=(self.length,0.2,self.length), color=color.red)
        wallUp   = box(pos=vector(0,self.length/2.,0), size=(self.length,0.2,self.length), color=color.white)
        wallL    = box(pos=vector(-self.length/2.,0,0), size=(0.2,self.length,self.length), color=color.blue)       

        self.unit_target     = self.target_vector / np.linalg.norm(self.target_vector)
        self.target_position = np.array(self.unit_target) * self.max_factor * self.force_scale 
        self.ball            = sphere(pos=[0,0,0], radius=self.ball_rad, color=self.ball_color)
        self.target          = sphere(pos=self.target_position, radius=self.target_rad, color=self.target_color) 
        self.shadow_cursor   = ring(pos=[0,-self.length/2,0], axis=(0,10,0), radius=self.ball_rad, thickness=1, color=[0.25, 0.25, 0.25])
        self.shadow_target   = ring(pos=[self.target_position[0],-self.length/2, self.target_position[2]], axis=(0,10,0), radius=self.ball_rad, thickness=1, color=[0.25, 0.25, 0.25])

        
    def run(self):
        while True:
            msg = CMessage()
            rcv = self.mod.ReadMessage(msg, -1)
            if rcv == 1:
                msg_type = msg.GetHeader().msg_type
                dest_mod_id = msg.GetHeader().dest_mod_id
                if  msg_type == MT_EXIT:
                    if (dest_mod_id == 0) or (dest_mod_id == self.mod.GetModuleID()):
                        print 'Received MT_EXIT, disconnecting...'
                        self.mod.SendSignal(rc.MT_EXIT_ACK)
                        self.mod.DisconnectFromMMM()
                        self.pi.ser.close()
                        break;
                elif msg_type == rc.MT_PING:
                    respond_to_ping(self.mod, msg, 'RTFT_iso')
                else:
                    self.process_msg(msg)
                    
    def process_msg(self, in_msg):
        header = in_msg.GetHeader()
        if header.msg_type == rc.MT_FT_DATA:
            mdf = rc.MDF_FT_DATA()
            copy_from_msg(mdf, in_msg)
            rate(self.rate) 
            self.ball.pos = vector(mdf.F[0:3])
            self.shadow_cursor.pos = vector([mdf.F[0],-self.length/2, mdf.F[2]])
            self.unit_target = np.array(self.target_vector) / np.linalg.norm(self.target_vector)
            self.target_position = np.array(self.unit_target) * self.max_factor * self.force_scale
            self.target.pos = self.target_position
            self.shadow_target.pos   = [self.target_position[0],-self.length/2, self.target_position[2]]
            distance = [a-b for a,b in zip(self.ball.pos,self.target.pos)]
            if (distance[0]**2+distance[1]**2+distance[2]**2)**(1/2.) >= self.threshold and self.RTFT_display:
                self.ball.color = self.ball_color
                self.state = 0
            elif (distance[0]**2+distance[1]**2+distance[2]**2)**(1/2.) < self.threshold and self.RTFT_display :  
                if self.state == 0: # if previous sample was outside radius, and now we're inside...
                    self.start_hold = time.time()
                    self.state = 1
                    self.ball.color = color.orange
                else:
                    if time.time() > (self.start_hold + self.hold_time):
                        self.ball.color = color.green
                        self.target.visible = False
                        self.shadow_target.visible = False
                        self.state = 2
                        out_mdf = rc.MDF_FT_COMPLETE()
                        out_mdf.FT_COMPLETE = self.state
                        out_mdf.sample_header = mdf.sample_header
                        msg = CMessage(rc.MT_FT_COMPLETE)
                        copy_to_msg(out_mdf, msg)
                        self.mod.SendMessage(msg)
                    else:
                        self.state = 1
                        self.ball.color = color.orange
            else:
                self.state = -1
                
            if self.state == 2 and self.solo:           #if no executive file
                self.target.pos = [float(x) for x in [np.random.rand(1,1)*self.max_factor*self.force_scale , np.random.rand(1,1)*self.max_factor*self.force_scale , np.random.rand(1,1)*self.max_factor*self.force_scale ]]
                self.shadow_target.pos = [self.target.pos[0], -self.length/2, self.target.pos[2]]   
        
            sys.stdout.write("%7.4f, %5d, %16.2f\n" % (mdf.F[2], self.state, (self.start_hold + self.hold_time) - time.time()))
            #msg_str = "%7.4f   " * 6 + "\n"
            #sys.stdout.write(msg_str % (mdf.F[0], mdf.F[1], mdf.F[2],
            #                            mdf.T[0], mdf.T[1], mdf.T[2]))
            sys.stdout.flush()

        elif header.msg_type == rc.MT_RTFT_CONFIG:
            mdf = rc.MDF_RTFT_CONFIG()
            copy_from_msg(mdf, in_msg)
            self.max_factor = mdf.max_factor
            self.RTFT_display = mdf.RTFT_display
            self.target_vector = mdf.target_vector[:]
            self.ball.visible = mdf.cursor_visible
            self.target.visible = mdf.target_visible
            self.shadow_target.visible = mdf.shadow_target_visible
            self.shadow_cursor.visible = mdf.shadow_cursor_visible
            self.ball_color = [1,0,0]
            self.solo = False

  
if __name__ == "__main__":
    parser = ArgumentParser(description = 'Real-time display of force directions and target postions')
    parser.add_argument(type=str, dest='config')
    parser.add_argument(type=str, dest='mm_ip', nargs='?', default='127.0.0.1:7111')
    args = parser.parse_args()
    print("Using config file=%s, MM IP=%s" % (args.config, args.mm_ip))
    fd = RTFT(args.config, args.mm_ip)
    sys.exit(exit_status)
    print "Got here"
