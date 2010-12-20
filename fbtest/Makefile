CC=arm-linux-gcc
CFLAGS=-c -Wall
LDFLAGS=
SOURCES=fbtest.c
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=fbtest

all: $(SOURCES) $(EXECUTABLE) fbtest_install
	
fbtest_install: fbtest
	cp fbtest ~/dev/gxlinux/rootfs/home

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o $(EXECUTABLE)
