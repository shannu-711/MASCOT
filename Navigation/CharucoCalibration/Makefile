CC = 'g++'
PROJECT = CharucoCalibration
SRC = CharucoCalibration.cpp

LIBS = `pkg-config opencv4 --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)