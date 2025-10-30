# Makefile for compiling and running the test suite.

# Compiler and flags
CXX = g++
# Use C++20 for concepts and other modern features.
# -I. adds the root directory to the include path.
CXXFLAGS = -std=gnu++23 -I. -Iexternal -Wall -Wextra -Werror -g
# Directories
BUILD_DIR = build

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
TEST_SRCS := $(wildcard tests/*.cpp)

# Object files
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
TEST_OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SRCS))

# Target executable
TARGET = $(BUILD_DIR)/test_runner

# Default target
all: $(TARGET)

# Link the test runner
$(TARGET): $(OBJS) $(TEST_OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(TEST_OBJS)

# Compile source files
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@
	$(CXX) $(CXXFLAGS) -M $< -MT $@ > $(@:.o=.d)

# Compile test files
$(BUILD_DIR)/tests/%.o: tests/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@
	$(CXX) $(CXXFLAGS) -M $< -MT $@ > $(@:.o=.d)


# Run the tests
test: all
	./$(TARGET)

# Clean up build artifacts
clean:
	rm -rf $(BUILD_DIR)

format:
	@find ./ \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.ino' \) ! -path './external/*' ! -path './.*/*' -exec clang-format --Wno-error=unknown -i '{}' \;

-include $(BUILD_DIR)/*.d

.PHONY: all test clean format
