# Compiler and flags
CXX = g++
# CXXFLAGS = -g -Wall -Wextra -O3 -std=c++17 -IInclude 
CXXFLAGS = -g -Wall -Wextra -O3 -std=c++17 -IInclude -fopenmp

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# Output binary name
TARGET = $(BIN_DIR)/RRTstar

# Source files and corresponding object files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))

# Default target
all: $(TARGET)

# Link the final executable
$(TARGET): $(OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OPTIONS) $^ -o $@

# Compile source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(OPTIONS) -c $< -o $@

# Clean target
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Phony targets
.PHONY: all clean
