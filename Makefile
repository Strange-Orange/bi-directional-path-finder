CC = g++
SRC = ./src
OBJ = ./obj
SRCS = $(wildcard $(SRC)/*.cc)
OBJS = $(patsubst $(SRC)/%.cc, $(OBJ)/%.o, $(SRCS))
CFLAGS = -c -g -Wall -std=c++17
LDLIBS = -pthread -lSDL2
OUTPUT = ./bin/bi-dir-path-finder.out

# Link
$(OUTPUT): $(OBJS)
	@mkdir -p ./bin
	$(CC) $^ $(LDLIBS) -o $(OUTPUT)

# Compile
$(OBJ)/%.o: $(SRC)/%.cc
	@mkdir -p $(OBJ)
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJ)/*.o ./bin/*.out
