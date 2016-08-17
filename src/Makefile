# Makefile to compiler ODE-0.12 program in Linux

CC = gcc -Wall -g -DdDOUBLE -DdTRIMESH_ENABLED
# Change "TARGET" to your program name without extension
#TARGET = ex_spring_constant
#TARGET = main
#TARGET = arm3
TARGET = hopper2
OBJS = $(TARGET).o
HEADER = 
LIBS = -L/usr/lib -L/usr/local/lib -L/usr/X11R6/lib 
INDS = -I. -I/usr/include/ode -I/usr/local/include/drawstuff -I/usr/X11R6/include
OPTS = -lode -ldrawstuff -lGL -lGLU -lglut -lX11

$(TARGET):$(OBJS) $(HEADER)
	$(CC) -o $(TARGET) $(OBJS) $(LIBS) $(INDS) $(OPTS)

$(TARGET).o:$(TARGET).cpp $(HEADER)
	$(CC) -c $(TARGET).cpp  $(LIBS) $(INDS)

clean:
	rm $(TARGET) $(OBJS) *.*~  *~
