# Makefile for compiling and running the test suite.

# Compiler and flags
CXX = g++
# Use C++20 for concepts and other modern features.
# -I. adds the root directory to the include path.
CXXFLAGS = -std=c++20 -I. -Wall -Wextra -g

# Source files
# List all .cpp files from the project root that contain core logic.
SRCS = \
    DptHelpers.cpp \
    command.cpp \
    firecontrol.cpp \
    state.cpp \
    target.cpp \
    target_selection.cpp

# Test files
TEST_SRCS = \
    test/main.cpp \
    test/test_vector.cpp \
    test/test_approximate_math.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)
TEST_OBJS = $(TEST_SRCS:.cpp=.o)

# Target executable
TARGET = test_runner

# Default target
all: $(TARGET)

# Link the test runner
$(TARGET): $(OBJS) $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(TEST_OBJS)

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files
test/%.o: test/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run the tests
run: all
	./$(TARGET)

# Clean up build artifacts
clean:
	rm -f $(OBJS) $(TEST_OBJS) $(TARGET)

.PHONY: all run clean