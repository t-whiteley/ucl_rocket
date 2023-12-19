CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g


SRC = airbrake.c IMU_sim.c
OBJ = $(SRC:.c=.o)
EXECUTABLE = comp

all: $(EXECUTABLE) run_python

$(EXECUTABLE): $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

output.csv: $(EXECUTABLE)
	./$(EXECUTABLE)

run_python: output.csv
	python data_vis.py

clean:
	rm -f $(OBJ) $(EXECUTABLE) output.csv