# Host unit test for firmware/DSPi/bt_proto.h (BT vendor wire formats)
#   make        build + run
CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
INCLUDES = -I../../firmware/DSPi
BIN = bt_vendor_test
.PHONY: all build run clean
all: run
build: $(BIN)
$(BIN): bt_vendor_test.c ../../firmware/DSPi/bt_proto.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BIN) bt_vendor_test.c
run: $(BIN)
	./$(BIN)
clean:
	rm -f $(BIN)
