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

        print("Using config file:", config_file)

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
        usbi2c = cfg.get_device("gpio")
        ##self.sensor = cfg.get_device("encoder")

                
        usbi2c.setup(0, usbi2c.OUT, usbi2c.PUSH_PULL)
        usbi2c.setup(1, usbi2c.OUT, usbi2c.PUSH_PULL)
        usbi2c.output(0, 0)
        usbi2c.output(1, 0)


        spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)

        self.motor = axis.axis_between(SPI = spi, SPI_CS = spi.I2CSPI_SS0, StepsPerUnit=1)
        #self.motor.Reset(KVAL_RUN = 0x29, KVAL_ACC = 0x39, KVAL_DEC = 0x39, FS_SPD = 0xFFFFFF)


        # Transition to newer axis class
        kvals = tefo_conf['tefo']['kval']
        self.motor.Setup(MAX_SPEED = tefo_conf['tefo']['speed'],
                       KVAL_ACC=kvals,
                       KVAL_RUN=kvals,
                       KVAL_DEC=kvals,
                       ACC = 100,
                       DEC = 100,
                       FS_SPD=3000,
                       STEP_MODE = axis.axis.STEP_MODE_1_16)


        self.motor.MaxSpeed(tefo_conf['tefo']['speed'])
        #self.motor.MoveWait(-1000)
        self.motor.Float()

        def_dir = False
        gpio_pins = [0,1]
        data = {
            'dirToHome': def_dir,
            'GPIO_pins': gpio_pins
            }

        #self.motor.search_range(False, usbi2c, gpio_pins)
        #sys.exit(0)


        (a,b) = self.motor.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        if b:
            self.motor.Move(tefo_conf['tefo'].get('release', 1000), 1, 0)
        if a:
            self.motor.Move(tefo_conf['tefo'].get('release', 1000), 0, 0)

        self.motor.Wait()

        (a,b) = self.motor.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        if self.motor.getStatus()['SW_F']:
            return("Je zmacknute tlacitku, ukoncuji")
            sys.exit(0)

        self.calib()

        data = None

        while True:
            #print self.sensor.get_angle(verify = True)

            #sys.stdout.write("RPS01A Angle: " + str(self.sensor.get_angle(verify = True)) + "\t\tMagnitude: " + str(self.sensor.get_magnitude())
            #    + "\tAGC Value: " + str(self.sensor.get_agc_value()) + "\tDiagnostics: " + str(self.sensor.get_diagnostics()) + "\r\n")
            #sys.stdout.flush()
            #print self.motor.getStatus()
            try:

                data = None
                data, addr = self.sock.recvfrom(1024)
                print("received message:", data, addr)
            except Exception as e:
                pass

            if data:
                miss = self.is_misscalibrated()
                ip, port = addr
                self.sock.sendto("ACK;%s;%s;\n\r" %(data.strip(), ip), addr)
                #
                #   'H' parameter - Home position defied in json file
                #
                if data[0] == 'H' :
                    self.motor.GoTo(self.tefo_conf['tefo']['home'], wait=True)
                    self.target = self.tefo_conf['tefo']['home']
                    self.motor.Wait()
                    self.sock.sendto("Home;%s;\n\r" %(miss), addr)
                    self.last_pos = self.sensor.get_angle(verify = False)
                    self.motor.Float()

                #
                #   'CMxxxx' parameter - calib and move to position in promile (0-1000)
                #
                elif 'CM' in data:
                    target = float(data[2:])
                    if target > 995: target = 995
                    if target < 15: target = 15
                    self.calib(target)
                    self.target = target
                    self.sock.sendto("CalibMove;%s;\n\r" %(miss), addr)

                #
                #   'Mxxxx' parameter - Move to position in promile(0-1000)
                #
                elif data[0] == 'M':
                    target = float(data[1:])
                    if target > 995: target = 995
                    if target < 15: target = 15
                    self.target = target
                    move = int(tefo_conf['tefo']['lenght']*target/1000)
                    self.motor.GoTo(move, wait=True)
                    self.motor.Wait()
                    self.sock.sendto("Move;%s;\n\r" %(miss), addr)
                    ##self.last_pos = self.sensor.get_angle(verify = False)
                    self.motor.Float()

                elif data[0] == 'C':
                    self.calib()

                elif data[0] == 'S':
                    miss = self.is_misscalibrated()
                    (a,b) = self.motor.validate_switch(usbi2c, gpio_pins)
                    position = self.motor.getPosition()
                    self.sock.sendto("%s;%s;%s;%s;%s;\n\r" %(miss, self.target, int(a), int(b), str(position)), addr)
                else:
                    print("neznamy prikaz")
            else:
                time.sleep(0.2)

        self.motor.Float()

    def is_misscalibrated(self):
        print(self.last_pos)
        #print self.sensor.get_angle(verify = False)
        diff = 0
        #diff = abs(float(self.last_pos) - self.sensor.get_angle(verify = False))
        if diff < 5:
            return False
        else:
            return diff

    def fancy_calib(self):
        print("Zacatek kalibrace")

        if not self.motor.getStatus()['SW_F']:
            (a,b) = self.motor.validate_switch()

        


    def calib(self, pos = None):
        #pokud je software nove zapnuty (nebo neni definovany 'pos'), tak se chci vycentrovat. Jinak se navratit na 'pos' argument
        print("Zacatek kalibrace")
        '''
        if not pos:
            #pos = self.tefo_conf['tefo']['home']
            pos = self.tefo_conf['tefo']['lenght']
            print("position obtained from cfg", pos)
        else:
            self.target = int(pos)
            #pos = int(self.tefo_conf['tefo']['lenght']*pos/1000)
            pos = int(self.tefo_conf['tefo']['lenght']/2)
        '''
        
        pos = self.tefo_conf['tefo']['lenght']
        print self.motor.getStatus()
        self.motor.MaxSpeed(self.tefo_conf['tefo']['home_speed'])

        print("Pohyb Move o", pos)
        self.motor.Move(pos)
        self.motor.Wait()

        ## TADdy JE Prvni NARAZ DO KONCAKU
        print('Move na koncak dokoncen')

        time.sleep(0.5)

        move = int(self.tefo_conf['tefo']['lenght']*self.tefo_conf['tefo']['home']/1000)


        self.motor.Float()
        self.motor.ResetPos()
        #self.motor.MaxSpeed(self.tefo_conf['tefo']['speed'])

        self.motor.MaxSpeed(100)
        #self.motor.MinSpeed(self.tefo_conf['tefo'].get('release_speed', 200), LSPD_OPT = False)

        #self.motor.ReleaseSW(True)
        #print("cekam na uvolneni")
        #self.motor.Wait()
        #print('tlacitko uvolneno')
        #time.sleep(0.5)
        #self.motor.ResetPos()
        print(self.motor.getStatus())

        #sys.exit(0)

        print("Pohyb na polohu", self.tefo_conf['tefo']['home'])
        self.motor.GoTo(move, wait=True)
        self.motor.Wait()
        time.sleep(0.5)
        self.motor.MaxSpeed(self.tefo_conf['tefo']['home_speed'])
        self.motor.GoTo(move, wait=True)
        self.motor.Float()

        self.target = int(self.tefo_conf['tefo']['home'])
        self.last_pos = 0
        #self.last_pos = self.sensor.get_angle(verify = True)

        #self.motor.MinSpeed(20)
        self.motor.MaxSpeed(self.tefo_conf['tefo']['speed'])
        print("konec kalibrace", self.last_pos)



def main():
    f = focuser()

if __name__ == '__main__':
    main()
