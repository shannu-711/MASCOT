CC = 'g++'
PROJECT = ArucoDetect1Cam
SRC = ArucoDetect1Cam.cpp

LIBS = `pkg-config opencv4 --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)