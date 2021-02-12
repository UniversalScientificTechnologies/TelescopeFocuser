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

        self.tefo_type = tefo_conf['tefo'].get('TEFO_type', 'dual')

        cfg = config.Config(i2c=tefo_conf['pymlab']['i2c'],  bus=tefo_conf['pymlab']['bus'])

        self.last_pos = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((tefo_conf['connection']['ip'], tefo_conf['connection']['port']))
        self.sock.setblocking(0)

        cfg.initialize()
        spi = cfg.get_device("spi")
        self.usbi2c = cfg.get_device("gpio")
        ##self.sensor = cfg.get_device("encoder")

        self.usbi2c.setup(0, self.usbi2c.OUT, self.usbi2c.PUSH_PULL)
        self.usbi2c.setup(1, self.usbi2c.OUT, self.usbi2c.PUSH_PULL)
        self.usbi2c.output(0, 0)
        self.usbi2c.output(1, 0)

        spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)
        self.motor=axis.axis_between(SPI=spi, SPI_CS=spi.I2CSPI_SS0, StepsPerUnit=1)
        #self.motor.Reset(KVAL_RUN=0x29, KVAL_ACC=0x39, KVAL_DEC=0x39, FS_SPD=0xFFFFFF)

        kvals = tefo_conf['tefo']['kval']
        self.MAX_SPEED = tefo_conf['tefo']['speed']
        self.KVAL_ACC = kvals
        self.KVAL_RUN = kvals
        self.KVAL_DEC = kvals
        self.ACC = 100
        self.DEC = 100
        self.FS_SPD = 3000

        self.direction = tefo_conf['tefo']['dir']
        self.dirToHome = int (not self.direction);

        self.setup_motor()

        def_dir = False
        self.gpio_pins = [0,1]
        data = {
            'dirToHome': def_dir,
            'self.gpio_pins': self.gpio_pins
            }

        #self.motor.search_range(False, self.usbi2c, self.gpio_pins)
        #sys.exit(0)

        #self.motor.Move(2000, 0)
        #self.motor.Wait()
        #self.motor.SoftStop ()
        #self.motor.Float()
        #sys.exit(1)


        # (a,b) = self.motor.validate_switch(self.usbi2c, self.gpio_pins)
        (a, b) = self.get_switch()
        print("Endstop states:", a, b)


        # Solving a problem, when the initial position is not within the allowed range
        # (i.e. the endswitch is closed).
        # We discovered that the behaviour of the unit is a bit non-standard in such case. 
        # Some setup parameters seem to be ignored, including the microstep mode.
        # It's necessary to safely move into operational range (where the endswitch is open)
        # and repeat the initialization.
        if a or b:
            print("Moving slowly towards the operational range (releaseSW)")
            if b:   # the default case, when only one endswitch is present
                self.motor.ReleaseSW(direction=self.direction, ACT=False)
            elif a:
                self.motor.ReleaseSW(direction=self.dirToHome, ACT=False)
            if self.motor_wait_stop (timeout=60, message="ERROR: timeout while attempting to get into the operational range"):
                sys.exit(1)
            print("releaseSW should be finished")

            time.sleep(0.2)
            self.motor.MaxSpeed(self.tefo_conf['tefo']['release_speed'])
            time.sleep(0.2)

            # We move a bit farther to get from a "greyzone" where the endswitch could flutter...
            # The fluttering of the endswitch (common especially for IR sensors) can interrupt 
            # this process, so we may need to run it repeatedly.
            attempt = 0
            while True:
                print("inicialization release, Move attempt: ", attempt)
                self.motor.Move(tefo_conf['tefo'].get('release', 1000), self.direction, False, 'steps')
                if self.motor_wait_stop(timeout=10, message="timeout Move"):
                    sys.exit(1)
                if abs(self.motor.getPosition()) >= abs(tefo_conf['tefo'].get('release', 1000)):
                    print("success: ", abs(self.motor.getPosition()))
                    break
                else:
                    print("wrong position, probably interrupted by a flutter of the endswitch: ", abs(self.motor.getPosition()))
                attempt = attempt + 1
                if attempt > 10:
                    print("FAIL: unable to realize an initial release...")
                    sys.exit(1)
                time.sleep(0.2)

            time.sleep(0.5)

            # And repeat the initialization...
            self.setup_motor()


        # (a,b) = self.motor.validate_switch(self.usbi2c, self.gpio_pins)
        (a,b) = self.get_switch()
        print("Endstop states:", a, b)

        if self.motor.getStatus()['SW_F']:
            print("Safety check: the endstop is still closed, exiting...")
            sys.exit(0)

        if self.tefo_conf['tefo'].get("calib", True):
            self.calib()

        data = None

        while True:
            #print(self.sensor.get_angle(verify=True))

            #sys.stdout.write("RPS01A Angle: " + str(self.sensor.get_angle(verify=True)) + "\t\tMagnitude: " + str(self.sensor.get_magnitude())
            #    + "\tAGC Value: " + str(self.sensor.get_agc_value()) + "\tDiagnostics: " + str(self.sensor.get_diagnostics()) + "\r\n")
            #sys.stdout.flush()
            #print(self.motor.getStatus())
            try:

                data = None
                data, addr = self.sock.recvfrom(1024)
                data = data.decode()
                print("received message:", data, addr)
            except Exception as e:
                pass

            if data:
                miss = self.is_misscalibrated()
                ip, port = addr
                message = "ACK;%s;%s;\n\r" %(data.strip(), ip)
                self.sock.sendto(message.encode(), addr)
                #
                #   'H' parameter - Home position defined in json file
                #
                if data[0] == 'H' :
                    move = int(tefo_conf['tefo']['lenght']*self.tefo_conf['tefo']['home']/1000)
                    if self.direction == 0:
                        move = -move
                    self.motor.GoTo(move)
                    self.motor_wait_stop(timeout=30, message="timeout GoTo")
                    self.target = self.tefo_conf['tefo']['home']
                    message = "Home;%s;\n\r" %(miss)
                    self.sock.sendto(message.encode(), addr)
                    #self.last_pos = self.sensor.get_angle(verify=False)
                    self.motor.Float()

                #
                #   'CMxxxx' parameter - calib and move to position in promile (0-1000)
                #
                elif 'CM' in data:
                    target = float(data[2:])
                    if target > 1000: target = 1000
                    if target < 1: target = 1
                    self.calib(target)
                    message = "CalibMove;%s;\n\r" %(miss)
                    self.sock.sendto(message.encode(), addr)

                #
                #   'Mxxxx' parameter - Move to position in promile(0-1000)
                #
                elif data[0] == 'M':
                    target = float(data[1:])
                    if target > 1000: target = 1000
                    if target < 1: target = 1
                    self.target = target
                    move = int(tefo_conf['tefo']['lenght']*target/1000)
                    if self.direction == 0:
                        move = -move
                    self.motor.GoTo(move)
                    self.motor_wait_stop(timeout=60, message="timeout GoTo")
                    message = "Move;%s;\n\r" %(miss)
                    self.sock.sendto(message.encode(), addr)
                    ##self.last_pos = self.sensor.get_angle(verify = False)
                    self.motor.Float()

                elif data[0] == 'C':
                    self.calib()

                elif data[0] == 'S':
                    miss = self.is_misscalibrated()
                    (a,b) = self.motor.validate_switch(self.usbi2c, self.gpio_pins)
                    position = self.motor.getPosition()
                    message = "%s;%s;%s;%s;%s;\n\r" %(miss, self.target, int(a), int(b), str(position))
                    self.sock.sendto(message.encode(), addr)

                elif 'STOP' in data or 'stop' in data:
                    self.motor.SoftStop()
                    time.sleep(0.5)
                    self.motor.Float()

                else:
                    print("unknown command")
            else:
                time.sleep(0.2)

        self.motor.Float()

    def is_misscalibrated(self):
        print(self.last_pos)
        #print(self.sensor.get_angle(verify=False))
        diff = 0
        #diff = abs(float(self.last_pos) - self.sensor.get_angle(verify=False))
        if diff < 5:
            return False
        else:
            return diff

    def fancy_calib(self):
        print("Calibration started")

        if not self.motor.getStatus()['SW_F']:
            (a,b) = self.motor.validate_switch()




    def calib(self, pos = None):
        # The calibration is concluded by a movement to 'pos' (or to 'home' position, when 'pos' is not set).
        print("Calibration started")

        if not pos:
            pos = self.tefo_conf['tefo']['home']
            print("position obtained from cfg", pos)

        print(self.motor.getStatus())
        self.motor.MaxSpeed(self.tefo_conf['tefo'].get('home_speed', 100))

        print("Movement GoUntil")
        #self.motor.GoUntil(self.dirToHome, 450)
        #self.motor.GoUntil(self.dirToHome, 28)
        self.motor.GoUntil(self.dirToHome, self.tefo_conf['tefo'].get('home_speed', 100))
        if self.motor_wait_stop(timeout=60, message="timeout GoUntil"):
            sys.exit(1)

        ## HERE is the FIRST hit to ENDSWITCH
        print('GoUntil to endswitch finished')

        time.sleep(0.5)

        self.motor.ReleaseSW(direction=self.direction, ACT=False)
        print("waiting for release")
        if self.motor_wait_stop(timeout=30, message="timeout ReleaseSW"):
            sys.exit(1)
        print('endswitch released')
        time.sleep(0.5)

        print(self.motor.getStatus())

        # Move yet further by an offset, to overcome the hysteresis area (where a fluttering can occure, 
        # which could endanger the prospective recalibration process)...
        print('calibration offset...')
        self.motor.MaxSpeed(self.tefo_conf['tefo'].get('release_speed', 200))
        move = self.tefo_conf['tefo'].get('calib_release', 100)
        if self.direction == 0:
            move = -move
        self.motor.GoTo(move)
        if self.motor_wait_stop (timeout=5, message="timeout GoTo"):
            sys.exit(1)
        print(self.motor.getStatus())

        time.sleep(0.5)
        self.motor.Float()
        self.motor.ResetPos()
        print(self.motor.getStatus())

        #sys.exit(0)

        self.motor.MaxSpeed(self.tefo_conf['tefo']['speed'])
        move = int(self.tefo_conf['tefo']['lenght']*pos/1000)
        if self.direction == 0:
            move = -move
        time.sleep(0.5)

        print("Movement to position", pos)
        self.motor.GoTo(move)
        if self.motor_wait_stop(timeout=40, message="timeout GoTo"):
            sys.exit(1)

        self.target = int(pos)
        self.last_pos = 0
        #self.last_pos = self.sensor.get_angle(verify=True)

        #self.motor.MinSpeed(20)
        #self.motor.MaxSpeed(self.tefo_conf['tefo']['speed'])
        print("Calibration finished ", self.last_pos)

    def get_switch(self):
        if self.tefo_type == 'dual':
            return self.motor.validate_switch(self.usbi2c, self.gpio_pins)
        elif self.tefo_type == 'single':
            return (self.motor.getStatus()['SW_F'], self.motor.getStatus()['SW_F'])

    def setup_motor(self):
        # Transition to newer axis class
        self.motor.Float()
        time.sleep(0.5)
        self.motor.Setup(MAX_SPEED = self.MAX_SPEED,
                       KVAL_ACC = self.KVAL_ACC,
                       KVAL_RUN = self.KVAL_RUN,
                       KVAL_DEC = self.KVAL_DEC,
                       ACC = self.ACC,
                       DEC = self.DEC,
                       FS_SPD = self.FS_SPD,
                       STEP_MODE = axis.axis.STEP_MODE_1_16)
                       #STEP_MODE = axis.axis.STEP_MODE_FULL)

        time.sleep(0.1)

        # Set external switch behaviour (to not provide HardStop interrupt)
        self.motor.setConfig(EN_VSCOMP=0b0, SW_MODE=0b1)
        # ten VSCOMP jsem mel puvodne zapnutej (0b1), ale na WF delal hroznej rachot, tak jsem ho zatim vypnul, budu to jeste testovat, az se opravi fokuser na padesatce

        time.sleep(0.5)
        self.motor.Float()
        self.motor.ResetPos()
        time.sleep(0.5)

    def motor_wait_stop(self, timeout=5, message="motor timeout"):
        self.motor.Wait(maximal_time=timeout, print_pos=False)
        if self.motor.IsBusy():
            self.motor.SoftStop()
            print(message)
            time.sleep(0.5)
            self.motor.Float()
            return 1
        else:
            return 0


def main():
    f = focuser()

if __name__ == '__main__':
    main()
