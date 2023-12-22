CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -std=c11 -g
CXXFLAGS = -Wall -Wextra -std=c++11 -g -I thirdparty/

SRC_C = airbrake.c IMU_sim.c
SRC_CPP = kf.cpp
OBJ = $(SRC_C:.c=.o) $(SRC_CPP:.cpp=.o)
EXECUTABLE = comp

all: $(EXECUTABLE) run_python

$(EXECUTABLE): $(OBJ)
	$(CXX) $(OBJ) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

output.csv: $(EXECUTABLE)
	./$(EXECUTABLE)

run_python: output.csv
	python data_vis.py

clean:
	rm -f $(OBJ) $(EXECUTABLE) output.csv
