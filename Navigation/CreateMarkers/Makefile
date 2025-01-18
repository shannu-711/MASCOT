CC = 'g++'
PROJECT = CreateMarkers
SRC = CreateMarkers.cpp

LIBS = `pkg-config opencv4 --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)