CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic -pthread
LDFLAGS ?= -pthread
TARGET ?= cliffscanner-edge
SRC := $(wildcard src/*.cpp)
OBJ := $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJ) $(LDFLAGS)

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -Iinclude -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

install: $(TARGET)
	install -D -m 0755 $(TARGET) $(DESTDIR)/opt/cliffscanner/bin/$(TARGET)

.PHONY: all clean install
