# Makefile for compiling and running the test suite.

# Compiler and flags
CXX = g++
# Use C++20 for concepts and other modern features.
# -I. adds the root directory to the include path.
CXXFLAGS = -std=gnu++23 -I. -Wall -Wextra -g

# Source files
# List all .cpp files from the project root that contain core logic.
SRCS = \
    command.cpp \
    command_queue.cpp \
    firecontrol.cpp \
    state.cpp \
    target.cpp \
    target_selection.cpp

# Test files
TEST_SRCS = \
    tests/main.cpp \
    tests/test_command.cpp \
    tests/test_firecontrol.cpp \
    tests/test_target_selection.cpp \
    tests/test_approximate_math.cpp \
    tests/test_serializer.cpp \
    tests/test_vector.cpp

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
tests/%.o: tests/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run the tests
run: all
	./$(TARGET)

# Clean up build artifacts
clean:
	rm -f $(OBJS) $(TEST_OBJS) $(TARGET)

format:
	@find ./ \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.ino' \) ! -name 'LD2450.*' ! -path './fpm/*' ! -path './.*/*' -exec clang-format --Wno-error=unknown -i '{}' \;

.PHONY: all run clean format