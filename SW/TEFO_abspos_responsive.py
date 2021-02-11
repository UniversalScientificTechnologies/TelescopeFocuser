#!/usr/bin/python

#import logging
#logging.basicConfig(level=logging.DEBUG)

import sys
import time
import axis
import socket
import select
from pymlab import config
import json
import math


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

        # An internal "pointer" to the ongoing action. After it's finished, it will be chanded 
        # to self.action_next ('None' means everything is completed).
        # There is a special case of action "calibrate", which consists of 3 subsequent 
        # sub-actions (and the switching to action_next will be performed only when the last 
        # one of them is completed).
        # Possible values: None, finished, move, stop, calibrate, calibrate1, calibrate2, calibrate3.
        self.action_now = None
        self.action_next = None

        # Timeout of the ongoing action (more specifically, the timestamp when it will expire)
        self.action_timeout = None

        # Result of the last action (ok = 0, fail/timeout = 1)
        self.lastaction_result = None

        self.last_set_position = None

        # A moment, at which the ongoing action should finish.
        self.expected_moveend_time = 0

        self.last_command_id = None

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
                if self.motor_wait_stop (timeout=10, message="timeout Move"):
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
            self.calibrate()

        data = None
        command = None

        while True:
            #print("while run...")
            #print self.sensor.get_angle(verify=True)

            #sys.stdout.write("RPS01A Angle: " + str(self.sensor.get_angle(verify=True)) + "\t\tMagnitude: " + str(self.sensor.get_magnitude())
            #    + "\tAGC Value: " + str(self.sensor.get_agc_value()) + "\tDiagnostics: " + str(self.sensor.get_diagnostics()) + "\r\n")
            #sys.stdout.flush()
            #print self.motor.getStatus()

            # The select is used very minimalistically here, practically it's an equivalent of
			# a simple "sleep" method with an advantage of a prompt reaction to an incoming 
			# message.
            inputready,outputready,exceptready = select.select([self.sock],[],[],0.2)

            try:
                data = None
                data, addr = self.sock.recvfrom(1024)
                print("received message:", data, addr)
            except Exception as e:
                pass

            if data:
                try:
                    command = None
                    command_id, command = data.split(' ')
                    print("split result:", command_id, command)
                except Exception as e:
                    pass

            if command:
                #miss = self.is_misscalibrated()
                #ip, port = addr
                #self.sock.sendto("ACK;%s;%s;\n\r" %(data.strip(), ip), addr)

                if command_id == self.last_command_id:
                    request_status = "duplicity"

                #
                #   'S' parameter - get status
                #
                elif command[0] == 'S':
                    request_status = "accepted"

                #
                #   'C' parameter - (re)calibrate
                #
                elif command[0] == 'C':
                    if self.action_now is not None and "calibrate" in self.action_now:
                        request_status = "wrong"
                    else:
                        request_status = "accepted"
                        if self.action_now == "stop" or self.action_now == "finished":
                            self.action_next = "calibrate"
                        else:
                            self.calibrate()

                #
                #   'CMxxxx' parameter - calibrate and move to position (1-1000)
                #
                elif 'CM' in command:
                    if self.action_now is not None and "calibrate" in self.action_now:
                        request_status = "wrong"
                    else:
                        request_status = "accepted"
                        target = int(command[2:])
                        if target > 1000: target = 1000
                        if target < 1: target = 1
                        if self.action_now == "stop" or self.action_now == "finished":
                            self.last_set_position = target
                            self.action_next = "calibrate"
                        else:
                            self.calibrate(target)

                #
                #   'Mxxxx' parameter - move to position (1-1000)
                #
                elif command[0] == 'M':
                    if self.action_now is not None and "calibrate" in self.action_now:
                        request_status = "wrong"
                    else:
                        request_status = "accepted"
                        target = int(command[1:])
                        if target > 1000: target = 1000
                        if target < 1: target = 1
                        if self.action_now == "stop" or self.action_now == "finished":
                            self.last_set_position = target
                            self.action_next = "move"
                        else:
                            self.move(target)

                elif 'STOP' in command or 'stop' in command:
                    request_status = "accepted"
                    self.stop()

                else:
                    print("unknown command")
                    request_status = "wrong"

                position = int(self.motor.getPosition() / float(self.tefo_conf['tefo']['lenght']) * 1000.0)
                if self.direction == 0:
                    position = -position

                if self.motor.IsBusy():
                    if self.action_now == "move":
                        motor_status = "moving"
                    elif self.action_now is not None and "calibrate" in self.action_now:
                        motor_status = "calibrating"
                else:
                    motor_status = "idle"

                if self.action_now == "move":
                    time_to_moveend = self.expected_moveend_time - time.time()
                    if time_to_moveend < 0.0:
                        time_to_moveend = 0.0
                else:
                    time_to_moveend = 0.0

                self.sock.sendto("%s %s %s %s %s %s %s %s\n" %(command_id, command.rstrip(), request_status, motor_status, str(self.lastaction_result), str(position), str(self.last_set_position), str(time_to_moveend)), addr)

                self.last_command_id = command_id
                command = None

            else:
                # A place where all *_progress functions will be called
                # as well as timeouts being solved
                # as well as logic of switching next->now actions.
                if self.action_now is not None and time.time() > self.action_timeout:
                    print("Timeout!!!")
                    self.stop()
                    self.lastaction_result = 1  # fail

                if self.action_now == "finished":
                    if self.action_next:
                        if self.action_next == "move":
                            self.move();    # a target is taken from self.last_set_position
                        if self.action_next == "calibrate":
                            self.calibrate();   # a target is taken from self.last_set_position
                        self.action_next = None
                    else:
                        self.action_now = None
                elif self.action_now == "stop":
                    self.stop_progress()
                elif self.action_now == "move":
                    self.move_progress()
                elif self.action_now is not None and "calibrate" in self.action_now:
                    self.calibrate_progress()

                #time.sleep(0.2)

        self.motor.Float()

#    def is_misscalibrated(self):
#        print(self.last_pos)
#        #print self.sensor.get_angle(verify=False)
#        diff = 0
#        #diff = abs(float(self.last_pos) - self.sensor.get_angle(verify=False))
#        if diff < 5:
#            return False
#        else:
#            return diff


    def stop(self):
        self.motor.SoftStop()
        self.action_timeout = time.time() + 3
        self.action_now = "stop"
        self.action_next = None     # if the movement should follow, it's necessary to set it explicitly AFTER calling of the stop() function!!!
        #self.last_pos = None
        self.last_set_position = None

    def stop_progress(self):
        if not self.motor.IsBusy():     # i.e., this phase just ended
            self.action_now = "finished"


    def calibrate(self, pos=None):
        print("Calibration started")

        # The calibration is concluded by a movement to a position
        # (the first defined/known wins, in this order):
        # 'pos', last set position, 'home' position.
        if not pos:
            if self.last_set_position:
                pos = self.last_set_position
            else:
                pos = self.tefo_conf['tefo'].get("home", 100)
                print("position obtained from cfg: ", pos)

        status = self.motor.getStatus()
        print(status)

        if status['BUSY'] == 1 or status['MOT_STATUS'] != 0:    # if moving, then stop
            self.stop()
            self.action_next = "calibrate"  # postpone the calibration till the stop is finished... the target position will be propagated via self.last_set_position (the logic of operation's interrupts excludes the conflict with the preceding operation)
        else:
            self.motor.MaxSpeed(self.tefo_conf['tefo'].get('home_speed', 100))
            print("Movement GoUntil")
            self.action_timeout = time.time() + 60
            self.motor.GoUntil(self.dirToHome, self.tefo_conf['tefo'].get('home_speed', 100))
            time.sleep(0.1)

            self.action_now = "calibrate1"
            self.action_next = "move"

        self.last_set_position = pos


    def calibrate_progress(self):
        print("Calibration in progress")

        if self.action_now == "calibrate1":
            if not self.motor.IsBusy():     # i.e., this phase just ended
                ## HERE is the FIRST hit to ENDSWITCH
                print('GoUntil to endswitch finished')
                time.sleep(0.5)
                self.motor.ReleaseSW(direction=self.direction, ACT=False)
                print("waiting for release")
                self.action_timeout = time.time() + 30
                self.action_now = "calibrate2"
                time.sleep(0.1)

        if self.action_now == "calibrate2":
            if not self.motor.IsBusy():     # i.e., this phase just ended
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
                self.action_timeout = time.time() + 5
                self.action_now = "calibrate3"
                time.sleep(0.1)

        if self.action_now == "calibrate3":
            if not self.motor.IsBusy():     # i.e., this phase just ended
                print('calibration offset finished...')
                print(self.motor.getStatus())

                time.sleep(0.5)
                self.motor.Float()
                self.motor.ResetPos()
                print(self.motor.getStatus())

                self.action_now = "finished"


    def move(self, pos=None):
        print("Move started")

        # If pos is not defined, I want to (the typical case of a post-calibration movement)
        # move to last set position or 'home' position.
        if not pos:
            if self.last_set_position:
                pos = self.last_set_position
            else:
                pos = self.tefo_conf['tefo'].get("home", 100)
                print("position obtained from cfg: ", pos)

        status = self.motor.getStatus()
        print(status)

        if status['BUSY'] == 1 or status['MOT_STATUS'] != 0:    # if moving, then stop
            self.stop ()
            self.action_next = "move"  # postpone the movement till the stop is finished... the target position will be propagated via self.last_set_position (the logic of operation's interrupts excludes the conflict with the preceding operation)
        else:
            self.motor.MaxSpeed(self.MAX_SPEED)

            move = int(self.tefo_conf['tefo']['lenght']*pos/1000.0)
            if self.direction == 0:
                move = -move

            distance = abs(move - self.motor.getPosition())
            if distance > self.MAX_SPEED**2 / self.ACC:
                self.expected_moveend_time = time.time() + self.MAX_SPEED/self.ACC + distance/(self.MAX_SPEED*16.0)   # 16 coeff used because of the microsteps chosen, see setup, hardcoded for now
            else:
                self.expected_moveend_time = time.time() + 2* math.sqrt(distance/(self.ACC*16.0))     # 16 coeff used because of the microsteps chosen, see setup, hardcoded for now

            print("expecting end of move in: ", self.expected_moveend_time)

            self.action_timeout = self.expected_moveend_time + 3.0

            print(time.time(), "Begin of GoTo")

            self.motor.GoTo(move)
            time.sleep(0.1)
            self.action_now = "move"

        self.last_set_position = pos

    def move_progress(self):
        print(time.time(), "Move in progress...")

        if not self.motor.IsBusy():     # i.e., the movement is finished
            print(time.time(), 'Move finished')
            print(self.motor.getStatus())
            self.motor.Float()

            self.action_now = "finished"
            self.lastaction_result = 0  # success
            #TODO: possibly to add some check that we really reached the demanded position (and use it as a condition for setting of self.lastaction_result)?


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
        if self.motor.IsBusy ():
            self.motor.SoftStop ()
            print (message)
            time.sleep(0.5)
            self.motor.Float()
            return 1
        else:
            return 0


def main():
    f = focuser()

if __name__ == '__main__':
    main()
