BUILD_DIR = build
BINARY = BucketHeap 

all:
	cmake -B $(BUILD_DIR) -S .
	cmake --build $(BUILD_DIR)

run: all
	./$(BUILD_DIR)/$(BINARY)

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean all