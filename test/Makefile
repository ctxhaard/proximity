export PATH := /home/ctomasin/development/raspi/buildroot_out/host/usr/bin:$(PATH)

DEV ?= /dev/lcd0
user ?= root
REMOTE ?= uraspi

CFLAGS = -D DEV=$(DEV)
CXXFLAGS = -std=c++14

ifeq ($(NDEBUG),)
CFLAGS += -g -Wall -fmessage-length=0
else
CFLAGS += -O2 -g -Wall -fmessage-length=0
endif

CXXFLAGS += $(CFLAGS)

ARCH = arm

OBJS =


ifeq ($(MAKECMDGOALS),test)
CROSS_COMPILE =
OBJS += main.o
TARGET = lcd_test_x86
else
CROSS_COMPILE = arm-buildroot-linux-gnueabihf-
OBJS += main.o
TARGET = lcd_test
endif

CXX = g++
CC = gcc

LIBS =


.cpp.o:
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -o $@ -c $<


.c.o:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -o $@ -c $<


$(TARGET): $(OBJS)
	$(CROSS_COMPILE)$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

.PHONY: test
test: $(TARGET)
	./$(TARGET)
	
.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET) lcd_test_x86
	
# loads the command on the remote target
.PHONY: load	
load: $(TARGET)
	scp $(TARGET) $(user)@$(REMOTE):

all: $(TARGET)
