#!/usr/bin/python

#import logging
#logging.basicConfig(level=logging.DEBUG) 

import sys
import time
import axis
import socket
from pymlab import config

cfg = config.Config(
        i2c = {
            "port": 1,
        },

        bus = [
            { 
            "name":"spi", 
            "type":"i2cspi"
            },
            {
            "name": "encoder",
            "type": "rps01"
            }
        ],
)

UDP_IP = '127.0.0.1'
UDP_PORT = 5000



class focuser():
    lenght = 26500

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(0)


        cfg.initialize()
        spi = cfg.get_device("spi")
        self.sensor = cfg.get_device("encoder")


        print self.sensor.get_address()
        print self.sensor.get_zero_position() 

        spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)

        self.motor = axis.axis(SPI = spi, SPI_CS = spi.I2CSPI_SS0)
        self.motor.Reset(KVAL_RUN = 0x29, KVAL_ACC = 0x39, KVAL_DEC = 0x39, FS_SPD = 0xFFFFFF)

        self.motor.Float()
        self.motor.MaxSpeed(200)
        print self.motor.GetStatus()

        self.calib()

        data = None
        self.last_pos = None

        while True:
            #print self.sensor.get_angle(verify = True)

            sys.stdout.write("RPS01A Angle: " + str(self.sensor.get_angle(verify = True)) + "\t\tMagnitude: " + str(self.sensor.get_magnitude()) 
                + "\tAGC Value: " + str(self.sensor.get_agc_value()) + "\tDiagnostics: " + str(self.sensor.get_diagnostics()) + "\r\n")
            sys.stdout.flush()
            try:

                data = None
                data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
                print "received message:", data, addr
            except Exception as e:
                pass

            if data:
                if data[0] == 'M':
                    move = int(self.lenght*float(data[1:-1])/1000)
                    print "move to absolute position: %s" %(move)
                    self.motor.GoTo(move, wait=True)
                    self.sock.sendto("NewPositios", addr)
                    self.last_pos = self.sensor.get_angle(verify = False)
                if data[0] == 'C':
                    self.calib()


        self.motor.Float()
        


    def calib(self):
        print "Zacatek kalibrace"
        self.motor.MoveWait(self.lenght*1.2)
        time.sleep(0.5)
        self.motor.ResetPos()
        self.motor.Float()
        self.motor.MoveWait(-1000)
        #self.motor.MoveWait(-1000)
        self.motor.GoTo(self.lenght/2, wait=True)
        self.motor.Float()
        print "goto"
        self.last_pos = self.sensor.get_angle(verify = True)
        print "konec kalibrace"



def main():
    f = focuser()

if __name__ == '__main__':
    main()