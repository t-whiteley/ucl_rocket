CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11 -g -I thirdparty/

SRC_CPP = airbrake.cpp IMU_sim.cpp kf.cpp
OBJ = $(SRC_CPP:.cpp=.o)
EXECUTABLE = comp

all: $(EXECUTABLE) run_python

$(EXECUTABLE): $(OBJ)
	$(CXX) $(OBJ) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

output.csv: $(EXECUTABLE)
	./$(EXECUTABLE)

run_python: output.csv
	python data_vis.py

clean:
	rm -f $(OBJ) $(EXECUTABLE) output.csv
