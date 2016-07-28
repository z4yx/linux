TARGET=server

PREFIX=mipsel-unknown-linux-gnu-

CPP_FILES=$(wildcard *.cpp) $(wildcard ../common/*.cpp)
C_FILES=$(wildcard *.c) $(wildcard ../common/*.c)

OBJS= $(patsubst %.cpp,%.o,$(CPP_FILES))
OBJS+= $(patsubst %.c,%.o,$(C_FILES))

CFLAGS= -O2 -I../common
LDFLAGS= -static -O2 -s

all: $(TARGET)
$(TARGET): $(OBJS)
	$(PREFIX)gcc $(LDFLAGS) $(CFLAGS) $(OBJS) -o $@
%.o: %.cpp
	$(PREFIX)g++ $(CFLAGS) -c $< -o $@ -O2
%.o: %.c
	$(PREFIX)gcc $(CFLAGS) -c $< -o $@ -O2
clean:
	rm $(OBJS) $(TARGET)
