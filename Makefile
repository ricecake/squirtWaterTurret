# Makefile for compiling and running the test suite.

# Compiler and flags
CXX = g++

# Build mode: one of `safety`, `fast`, `debug`, `profile`.
# - `safety` (default): superset of `debug`, no optimizations, sanitizers, caller capture.
# - `fast`: optimized for quick pass/fail, -O2, no sanitizers.
# - `debug`: manual debugging: -g -O0, include caller capture (`LOGGER_CAPTURE_CALLER`).
# - `profile`: instrumented build for profiling (gprof/func instrumentation).
MODE ?= safety

# Common include paths
# Treat third-party headers as system headers to avoid strict warning errors.
# Place system include dirs before the project `-I.` so includes like "fpm/..."
# are resolved to system paths.
BASE_INCLUDES = -isystem fpm -isystem external -I.

# Common warning flags
# Use -Werror to treat warnings as errors for project code (not system includes)
BASE_WARN = -Wall -Wextra -Werror

# Mode-specific flags
ifeq ($(MODE),safety)
	# safety: disable optimizations for easiest debugging, enable sanitizers and caller capture
	# - Do NOT define a plain `DEBUG` macro here (it collides with common identifiers).
	MODE_FLAGS = -g -O0 -DLOGGER_CAPTURE_CALLER -fno-omit-frame-pointer \
		-fsanitize=address,leak,undefined,bounds,signed-integer-overflow \
		-fsanitize-undefined-trap-on-error \
		-fstack-protector-strong -D_FORTIFY_SOURCE=2
	# Extra warning-level checks useful for safety mode
	BASE_WARN += -Wshadow -Wconversion -Wsign-conversion -Wformat-security -Wnull-dereference -Wnon-virtual-dtor
else ifeq ($(MODE),fast)
	MODE_FLAGS = -O2
	# For very fast iterations, don't treat warnings as errors in fast mode
	BASE_WARN = -Wall -Wextra
else ifeq ($(MODE),debug)
	# Debug mode: keep symbols and no optimizations. Do not define plain `DEBUG`.
	MODE_FLAGS = -g -O0 -DLOGGER_CAPTURE_CALLER -fno-omit-frame-pointer
else ifeq ($(MODE),profile)
	# profiling: instrument functions and enable gprof support
	MODE_FLAGS = -g -O1 -pg -finstrument-functions -fno-omit-frame-pointer -DPROFILE
else
	$(error Unknown MODE '$(MODE)'. Supported: safety, fast, debug, profile)
endif

# Allow adding extra sanitizer flags from environment if desired
SANITIZER_EXTRA ?=

# Final CXXFLAGS
CXXFLAGS = -std=gnu++23 $(BASE_INCLUDES) $(BASE_WARN) $(MODE_FLAGS) $(SANITIZER_EXTRA)

# Silence make implicit rules if necessary
.SILENT:

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
