# Parser for headtracker razorAHRS
# compatible with razorAHRS firmware version 1.4.3

##### Program to read binary data stream of the [RazorAHRS](https://github.com/ptrbrtz/razor-9dof-ahrs) headtracker, written in C. Contains a parser and a virtual headtracker.


***

## Installation

#### Build
```bash
$ cmake . && make
```

#### Compile manually
```bash
$ gcc /src/main_razorAHRS.c -Wall -D_REENTRANT -lpthread -o /bin/main_razorAHRS
```

## Start

#### Preparing the tracker
Open the file Arduino/Razor_AHRS/Razor_AHRS.ino and select your Hardware under "USER SETUP AREA" / "HARDWARE OPTIONS". (If you have a SparkFun 9DOF Razor IMU and problems to decide wether you have version SEN-10125 or SEN-10736, have a close look at the Magnetometer module that is a little black brick under the printed label "9DOF RAZOR" and right sided to the GND-connector. If this module has 4 pins on each side it is the version SEN-10736, if it has 5 pins it is the version 10125.) Upload the firmware using Arduino.

Maybe you want to calibrate the tracker. It is recommended. Therefore you can find a good [tutorial](https://github.com/ptrbrtz/razor-9dof-ahrs/wiki/Tutorial#sensor-calibration) by Peter Bartz.


#### Run it (the reader is wrapped in a main function to demonstration):
```bash
$ ./bin/main_razorAHRS <port name> 
```

or

```bash
$ sh start_razorAHRS.sh
```

***

## Using parser in your code

All functions use the struct thread_parameter as argument.


#### Streaming modes

* STREAMINGMODE_ONREQUEST
* STREAMINGMODE_CONTINUOUS


#### Streaming formats

* STREAMINGFORMAT_ASCII
  * expecting data as string (undefined frame size)
* STREAMINGFORMAT_BINARY_FLOAT
  * expecting 3 floating point values (12 byte per frame)


### reader functions

* __Preparing the reader__
  * Returns the struct that is used as argument in all other functions.
  * Baudrate is usually 57600.
```C
struct thread_parameter *parameter = razorAHRS(speed_t baudRate, char* port, <streaming mode>, <streaming format> )
```

* __Starting the reader__
  * Returns 0 if successful, otherwise -1
```C
razorAHRS_start(struct_parameter *parameter)
```	

* __Stopping the reader__
  * returns 0 if succesful, otherwise -1
```C
razorAHRS_quit(struct_parameter *parameter)
```

* __Requesting a single frame__
 * ! works only with mode STREAMINGMODE_ONREQUEST !
 * returns 0 if successful, otherwise -1
```C
razorAHRS_request(struct_parameter *parameter)
```



### printing value functions

* __provide a value printer on stdout__
```C
razorPrinter(struct thread_parameter *parameter);
```

* __Starting the printer__
```C
razorPrinter_start(struct thread_parameter *parameter)
```

* __stopping the printer__
```C
razorPrinter_stop(struct thread_parameter *parameter)
```


***

## Virtual Tracker

Make sure you got socat:
```bash
$ sudo apt install socat 
```

Preparing the ports:
```bash
$ sh connect_virtual_ports.sh
```

It will:

1. create two connected ports via Socat
2. print out the port names



Read the printed port names (something like /dev/pts/1) and go on:


```bash
$ ./bin/main_razorAHRSvirtual <first port name, e.g. /dev/pts/1>
```

Because the virtual tracker prints out its values it is recommended to use another terminal for the reader otherwise it is nearly impossible to compare send and received data.
```bash
$ ./bin/main_razorAHRS <second port name, e.g. /dev/pts/2>
```

***


## Credits

__Authors__
* Christian Krüger
* Dennis Guse

__Institution__
* Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin

__Date__
* November 2016

__former version (source)__
* [https://github.com/ptrbrtz/razor-9dof-ahrs](https://github.com/ptrbrtz/razor-9dof-ahrs)






