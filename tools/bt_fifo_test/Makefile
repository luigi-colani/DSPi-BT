# Host unit test for firmware/DSPi/bt_fifo.h
#
#   make        build + run the test
#   make build  build only
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
# bt_fifo.h lives in the firmware source dir.
INCLUDES = -I../../firmware/DSPi

BIN = bt_fifo_test

.PHONY: all build run clean
all: run

build: $(BIN)

$(BIN): bt_fifo_test.c ../../firmware/DSPi/bt_fifo.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BIN) bt_fifo_test.c

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
