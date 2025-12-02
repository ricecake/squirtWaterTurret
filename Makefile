# Makefile for compiling and running the test suite.

# Compiler and flags
CXX = g++
# Use C++20 for concepts and other modern features.
# -I. adds the root directory to the include path.
BASE_INCLUDES = -isystem fpm -isystem external -I.
BASE_WARN = -Wall -Wextra -Werror -Wformat-security -Wnull-dereference -Wnon-virtual-dtor
TODO_WARN = -Wshadow -Wconversion -Wsign-conversion
FLAGS = -std=gnu++23 -g -O0\
		-fstack-protector-strong -D_FORTIFY_SOURCE=2 -fno-omit-frame-pointer \
		-fsanitize=address,leak,undefined,bounds,signed-integer-overflow \
		-fsanitize-undefined-trap-on-error

CXXFLAGS = -std=gnu++23 $(BASE_INCLUDES) $(BASE_WARN) $(TODO_WARN) $(FLAGS)

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

SKETCH = dualStepperDPTStartWithRadar.ino

# Test files
TEST_SRCS := $(wildcard tests/*.cpp)

# Object files
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
TEST_OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SRCS))
SKETCH_OBS = $(patsubst %.ino,$(BUILD_DIR)/%.o,$(SKETCH))
DEPS = $(OBJS:.o=.d) $(TEST_OBJS:.o=.d) $(SKETCH_OBS:.o=.d)

# Target executable
TARGET = $(BUILD_DIR)/test_runner
SKETCH_TARGET = $(BUILD_DIR)/mock_binary
sketch: $(SKETCH_TARGET)
testbin: $(TARGET)

# Default target
all: $(TARGET) $(SKETCH_TARGET)

# Link the test runner
$(TARGET): $(OBJS) $(TEST_OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(TEST_OBJS)

# Compile source files
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(@:.o=.d) -MT $@ -c $< -o $@

# Compile test files
$(BUILD_DIR)/tests/%.o: tests/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(@:.o=.d) -MT $@ -c $< -o $@

# Link the sketch
$(SKETCH_TARGET): $(OBJS) $(SKETCH_OBS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(SKETCH_TARGET) $(OBJS) $(SKETCH_OBS)

# Compile sketch
$(BUILD_DIR)/%.o: %.ino
	@mkdir -p $(@D)
	$(CXX) -x c++ $(CXXFLAGS) -MMD -MP -MF $(@:.o=.d) -MT $@ -c $< -o $@

# Run the tests
test: $(TARGET)
	./$(TARGET)

smoke: $(SKETCH_TARGET)
	./$(SKETCH_TARGET)

# Clean up build artifacts
clean:
	rm -rf $(BUILD_DIR)

format:
	@find ./ \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.ino' \) ! -path './external/*' ! -path './.*/*' -exec clang-format --Wno-error=unknown -i '{}' \;

-include $(DEPS)

.PHONY: all test smoke sketch testbin clean format
