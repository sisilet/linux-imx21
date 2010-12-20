CC=arm-linux-gcc
CFLAGS=-c -Wall
LDFLAGS=
SOURCES=i2ctest.c
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=i2ctest

all: $(SOURCES) $(EXECUTABLE) i2ctest_install
	
i2ctest_install: i2ctest
	cp i2ctest ~/dev/gxlinux/rootfs/home

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o $(EXECUTABLE)
