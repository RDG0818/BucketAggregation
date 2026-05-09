BUILD_DIR = build
BINARY = BucketHeap 

all:
	cmake -B $(BUILD_DIR) -S . -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	cmake --build $(BUILD_DIR)

run: all
	./$(BUILD_DIR)/$(BINARY)

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean all