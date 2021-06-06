CC = g++
EXE = main
OBJ = main.o load.o geometry.o
CFLAGS = -std=c++11 -Wall -g

all: $(OBJ)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJ)

%.o:%.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean: 
	rm $(OBJ) $(EXE)
