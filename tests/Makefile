HOST_CXX ?= g++
TEST_BINARY := .build/switch2_protocol_tests.exe

.PHONY: test clean
test: $(TEST_BINARY)
	./$(TEST_BINARY)

$(TEST_BINARY): switch2_protocol_tests.cpp ../mc_mitm/source/controllers/switch2_protocol.hpp
	mkdir -p .build
	$(HOST_CXX) -std=c++17 -O2 -Wall -Wextra -Werror $< -o $@

clean:
	rm -rf .build