# Host unit test for firmware/DSPi/bt_exclusion.h
#
#   make        build + run the test
#   make build  build only
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
# bt_exclusion.h lives in the firmware source dir.
INCLUDES = -I../../firmware/DSPi

BIN = bt_exclusion_test

.PHONY: all build run clean
all: run

build: $(BIN)

$(BIN): bt_exclusion_test.c ../../firmware/DSPi/bt_exclusion.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BIN) bt_exclusion_test.c

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
