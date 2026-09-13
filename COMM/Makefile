CC = gcc
CFLAGS = -Wall -O2
TARGET = robot_core
SRCS = robot_core.c pathfinding.c protocol.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c pathfinding.h protocol.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
