# TelescopeFocuser
Telescope focuser with position feedback - TEFO01B


![TEFO01B D50 telescope](DOC/SRC/img/instalace.JPG)

# Mlab modules
 * HBSTEP01B
 * I2CSPI01A
 * RPS01A
 * USBI2C (optional)
 * POWERSOURCE

# Dependencies
 * [PyMLAB](https://github.com/MLAB-project/pymlab) with [HIDAPI](https://github.com/trezor/cython-hidapi)
 * [Axis class](https://github.com/MLAB-project/axis)

# Versions
There are two versions of code - the "simple one" (`TEFO_abspos.py`) and the "responsive one" (`TEFO_abspos_responsive.py`).
They only differ in the communication protocol and general behaviour, the other aspects are the same.

## The "simple" version (```TEFO_abspos.py```)
### Behaviour
Every successully recieved message is immediately confirmed with a simple response (see below).
The reply of the commands is sent after the completion (e.g., after the movement was accomplished). The deamon is not accepting commands until the previous operation is finished.

### Communication (UDP messages)
 * ```H``` Go to home position (defined in config json), reply: ```Home;<miss_calib>;```
 * ```CMxxxx``` parameter - calibrate and move to position in promile (0-1000), reply: ```CalibMove;<miss_calib>;```
 * ```Mxxxx``` move to position (without calibration), reply: ```Move;<miss_calib>;```
 * ```C``` run calibration, None reply
 * ```S``` Status, answer is in format ```<miss_calib>;<target>```

After every succesful recieve of message, inmediately command is confirmed with ```ACK;<command>;<senders_ip>;```

## The "responsive" version (```TEFO_abspos_responsive.py```)
### Behaviour and communication (UDP messages)
 * The daemon is responsive, i.e. every command is executed "in background", the execution does not wait for the completion.
 * The daemon never initiates any message, the communication is based on strict question/answer scheme.
 * Answer to any command consists of:
   * The command which initiated the answer (including the numerical ID)
   * Status of the request (accepted/duplicity/wrong)
   * Actual state of a motor (idle/moving/calibrating)
   * Result of the last movement (0=ok/1=fail)
   * Actual position (works even within the movement)
   * Actually set target position (if it does not equal actual position, the state of a motor is idle and the last movement's result is ok, it's obvious that the unit had been restarted; it's up to a client to send a recalibration command)
   * Expected time [s] till the end of ongoing move operation, only useful when state of a motor is "moving" (the lower estimate, typically takes ~0.1-0.2s more time to finish).
 * To take care of lossy UDP protocol, we use a simple logic: the daemon remembers the last command's ID. When the new command has the same ID, we expect that the client didn't receive the response, so we don't perform any operation, just respond the "duplicity" status and report the actual state. The ID is generated by a client, the daemon remembers only the ID of last command and doesn't take care of IP address. Which is OK for sparse communication with typically one client.
 * We use these commands:
   | Command | Description |
   | --- | --- |
   | `M<number_int>` | move to position |
   | `CM<number_int>` | calibrate and move to position |
   | `C` | calibrate and move to last or "home" position |
   | `S` | send status report |
   | `STOP` | stop all movements (emergency command for testing purposes) |
 * Structure of client's commands:
   `<ID_int16> <command>`
 * Structure of the response:
   `<ID_int16> <command> <request-status_string> <motor-status_string> <last-movement-result_int> <actual-ABS-position_int> <set-ABS-position_int> <time-to-moveend_float>`
 * The interruptibility:
   * `STOP` quits any movement
   * the calibration may be interrupted by nothing but stop
   * move comands may be interrupted by any command
   * non-accepted commands (which violated the interruptibility rules) cause response status "wrong"


## Connection
```nc -u localhost <port>```

# Configuration
Configuration is done by config [.json]() file.

```json

{
    "pymlab":{
        "i2c":{
            "device": "hid",
            "serial": "003347BC"
        },
        "bus": [
            { 
                "name":"spi", 
                "type":"i2cspi"
            },
            {
                "name": "encoder",
                "type": "rps01"
            }
        ]
    },
    "tefo":{
        "dir": 0,
        "lenght": 3300,
        "home": 1650,
        "speed": 100,
        "_lenght": 26500,
        "_home": 13250,
        "home_speed": 20 
    },
    "connection":{
        "ip": "127.0.0.1",
        "port": 5000
    }
}

```
