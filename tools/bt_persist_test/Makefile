# Host unit test for Task 6 persistence (wire V29 bt-config + directory migration)
#
#   make        build + run the test
#   make build  build only
#   make clean
#
# SDK-free: compiles against firmware/DSPi/bulk_params.h + bt_persist.h only.

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
# bulk_params.h and bt_persist.h live in the firmware source dir.
INCLUDES = -I../../firmware/DSPi

BIN = bt_persist_test

.PHONY: all build run clean
all: run

build: $(BIN)

$(BIN): bt_persist_test.c ../../firmware/DSPi/bulk_params.h ../../firmware/DSPi/bt_persist.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BIN) bt_persist_test.c

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
