CC = 'g++'
PROJECT = ArucoDetect
SRC = ArucoDetect.cpp

LIBS = `pkg-config opencv4 --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)