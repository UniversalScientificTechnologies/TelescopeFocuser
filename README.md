# TelescopeFocuser
Telescope focuser with position feedback - TEFO01B


![TEFO01B D50 telescope](DOC/SRC/img/instalace.JPG)

# Mlab modules
 * HBSTEP01B
 * I2CSPI01A
 * RPS01A
 * USBI2C (otpinal)
 * POWERSOURCE

# Dependencies
 * [PyMLAB](https://github.com/MLAB-project/pymlab)
 * [Axis class](https://github.com/MLAB-project/axis)

# UDP messages
 * ```H``` Go to home position (defined in config json), reply: ```Home;<miss_calib>;```
 * ```CMxxxx``` parameter - calibrate and move to position in promile (0-1000), reply: ```CalibMove;<miss_calib>;```
 * ```Mxxxx``` move to position (without calibration), reply: ```Move;<miss_calib>;```
 * ```C``` run calibration, None reply
 * ```S``` Status, answer is in format ```<miss_calib>;<target>```

After every succesful recieve of message, inmediately command is confirmed with ```ACK;<command>;<senders_ip>;```


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
