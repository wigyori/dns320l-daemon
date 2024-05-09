CC?=gcc
CFLAGS?=-I. -I/usr/include
DEPS = dns320l.h
OBJ = dns320l-daemon.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

dns320l-daemon: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f *.o
