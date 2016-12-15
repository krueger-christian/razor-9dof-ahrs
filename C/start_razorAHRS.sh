#!/bin/sh 
if [ $# -ne 1 ]
 	then
 	echo "Call: $ sh start_razorAHRS.sh <port, e.g. /dev/ttyUSB0>"
 	exit 1 
else
 	echo "Allright master. I will provide the IMU data."

	./bin/main_razorAHRS $1

fi
