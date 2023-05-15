CFLAGS = -std=c99 -Wall -pedantic
LDFLAGS = -lraylib -lm

all: microplotter

microplotter: main.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

.PHONY: run clean

run: microplotter
	./microplotter

clean:
	rm -f microplotter

