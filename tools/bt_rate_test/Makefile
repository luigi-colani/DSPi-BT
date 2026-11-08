# Host unit test for firmware/DSPi/bt_rate.h
CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
INCLUDES = -I../../firmware/DSPi
BIN = bt_rate_test
.PHONY: all build run clean
all: run
build: $(BIN)
$(BIN): bt_rate_test.c ../../firmware/DSPi/bt_rate.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BIN) bt_rate_test.c
run: $(BIN)
	./$(BIN)
clean:
	rm -f $(BIN)
