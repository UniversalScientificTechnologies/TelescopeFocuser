#!/usr/bin/python

#import logging
#logging.basicConfig(level=logging.DEBUG) 

import sys
import time
import axis
import socket
from pymlab import config
import json


class focuser():

    def __init__(self):

        config_file = 'focuser.json'
        if len(sys.argv) == 2:
            config_file = sys.argv[1]

        print "Using config file:", config_file

        with open(config_file) as data_file:    
            self.tefo_conf = json.load(data_file)
        tefo_conf = self.tefo_conf

        cfg = config.Config(i2c = tefo_conf['pymlab']['i2c'],  bus = tefo_conf['pymlab']['bus'])

        self.last_pos = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((tefo_conf['connection']['ip'], tefo_conf['connection']['port']))
        self.sock.setblocking(0)


        cfg.initialize()
        spi = cfg.get_device("spi")
        self.sensor = cfg.get_device("encoder")


        print self.sensor.get_address()
        print self.sensor.get_zero_position() 

        spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)

        self.motor = axis.axis(SPI = spi, SPI_CS = spi.I2CSPI_SS0)
        self.motor.Reset(KVAL_RUN = 0x29, KVAL_ACC = 0x39, KVAL_DEC = 0x39, FS_SPD = 0xFFFFFF)

        self.motor.MaxSpeed(tefo_conf['tefo']['speed'])
        #self.motor.MoveWait(-1000)
        self.motor.Float()
        print self.sensor.get_angle(verify = False)
        print self.motor.GetStatus()

        self.calib()

        data = None

        while True:
            #print self.sensor.get_angle(verify = True)

            #sys.stdout.write("RPS01A Angle: " + str(self.sensor.get_angle(verify = True)) + "\t\tMagnitude: " + str(self.sensor.get_magnitude()) 
            #    + "\tAGC Value: " + str(self.sensor.get_agc_value()) + "\tDiagnostics: " + str(self.sensor.get_diagnostics()) + "\r\n")
            #sys.stdout.flush()
            print self.motor.getStatus()
            try:

                data = None
                data, addr = self.sock.recvfrom(1024)
                print "received message:", data, addr
            except Exception as e:
                pass

            if data:
                miss = self.is_misscalibrated()
                if data[:2] == 'MH' :
                    self.motor.GoTo(self.tefo_conf['tefo']['home'], wait=True)
                    self.motor.wait()
                    self.sock.sendto("Home, miss: %s" %(miss), addr)
                    self.last_pos = self.sensor.get_angle(verify = False)
                    self.motor.Float()

                elif data[0] == 'M':
                    target = float(data[1:])
                    self.target = target
                    if target > 1000: target = 1000
                    if target < 0: target = 0
                    move = int(tefo_conf['tefo']['lenght']*target/1000)
                    self.motor.GoTo(move, wait=True)
                    self.motor.wait()
                    self.sock.sendto("NewPositios, miss: %s" %(miss), addr)
                    self.last_pos = self.sensor.get_angle(verify = False)
                    self.motor.Float()

                elif data[0] == 'C':
                    self.calib()

                elif data[0] == 'S':
                    miss = self.is_misscalibrated()
                    self.sock.sendto("miss: %s, position: %s" %(miss, self.target), addr)
                else:
                    print "neznamy prikaz"

        self.motor.Float()
    
    def is_misscalibrated(self):
        print self.last_pos
        print self.sensor.get_angle(verify = False)
        diff = abs(float(self.last_pos) - self.sensor.get_angle(verify = False))
        if diff < 1:
            return False
        else:
            return diff


    def calib(self):
        print "Zacatek kalibrace"
        #self.motor.MoveWait(-1000)
        self.motor.MoveWait(self.tefo_conf['tefo']['lenght']*1.2)
        time.sleep(0.5)
        self.motor.Float()
        self.motor.ResetPos()
        self.target = self.tefo_conf['tefo']['home']
        self.motor.GoTo(self.tefo_conf['tefo']['home'], wait=True)
        self.motor.wait()
        self.motor.GoTo(self.tefo_conf['tefo']['home'], wait=True)
        self.motor.wait()
        self.motor.Float()
        self.last_pos = self.sensor.get_angle(verify = True)
        print "konec kalibrace", self.last_pos



def main():
    f = focuser()

if __name__ == '__main__':
    main()