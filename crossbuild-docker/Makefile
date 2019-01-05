C = /usr/bin/arm-linux-gnueabi-gcc-4.9
CPP = /usr/bin/arm-linux-gnueabi-g++-4.9
LD = /usr/bin/arm-linux-gnueabi-g++-4.9
CCFLAGS=-std=c++11 -Wall -O2 -I./ev3dev-lang-cpp -rdynamic
OBJ=
LIBS=-lstdc++ -lc -L/usr/lib/gcc/arm-linux-gnueabi/4.9 -L/lib/arm-linux-gnueabi -L/usr/lib/arm-linux-gnueabi -L./ev3dev-lang-cpp/build/ -lev3dev -lpthread -lm -rdynamic
 
# File names
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

prog: Console.o CYdLidar.o list_ports_linux.o lock.o serial.o unix_serial.o unix_timer.o ydlidar_driver.o ./CruiseControl.o CarControl.o Lidar.o CruiseControl.o main.o
	$(LD) -o YDLidarCar ./Console.o ./CYdLidar.o ./list_ports_linux.o ./lock.o ./serial.o ./unix_serial.o ./unix_timer.o ./ydlidar_driver.o ./CruiseControl.o ./CarControl.o ./Lidar.o ./main.o $(LIBS)

Console.o: ../ydlidar_driver/Console.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/Console.cpp

CYdLidar.o: ../ydlidar_driver/CYdLidar.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/CYdLidar.cpp
	
list_ports_linux.o: ../ydlidar_driver/list_ports_linux.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/list_ports_linux.cpp
	
lock.o: ../ydlidar_driver/lock.c
	$(C) $(CCFLAGS) -c ../ydlidar_driver/lock.c

serial.o: ../ydlidar_driver/serial.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/serial.cpp
	
unix_serial.o: ../ydlidar_driver/unix_serial.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/unix_serial.cpp
	
unix_timer.o: ../ydlidar_driver/unix_timer.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/unix_timer.cpp

ydlidar_driver.o: ../ydlidar_driver/ydlidar_driver.cpp
	$(CPP) $(CCFLAGS) -c ../ydlidar_driver/ydlidar_driver.cpp
	
CarControl.o: ../CarControl.cpp
	$(CPP) $(CCFLAGS) -c ../CarControl.cpp

Lidar.o: ../Lidar.cpp
	$(CPP) $(CCFLAGS) -c ../Lidar.cpp

CruiseControl.o: ../CruiseControl.cpp
	$(CPP) $(CCFLAGS) -c ../CruiseControl.cpp
	
main.o: ../main.cpp
	$(CPP) $(CCFLAGS) -c ../main.cpp

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)
