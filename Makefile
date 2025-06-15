.PHONY: all build test lint format clean

BUILD_DIR ?= build
PCL_DIR ?= $(CURDIR)/third_party/pcl
CMAKE_FLAGS ?= -DCMAKE_BUILD_TYPE=Release -DPCL_DIR=$(PCL_DIR)

# Find source files excluding third_party
SOURCES := $(shell find . -path ./third_party -prune -o \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print)

all: build

build:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)
	cmake --build $(BUILD_DIR)

# Run unit tests

test: build
	cd $(BUILD_DIR) && ctest --output-on-failure

# Lint source files with cpplint
lint:
	@cpplint $(SOURCES) || true

# Format source files with clang-format in place
format:
	@clang-format -i $(SOURCES)

# Remove build artifacts
clean:
	rm -rf $(BUILD_DIR)
