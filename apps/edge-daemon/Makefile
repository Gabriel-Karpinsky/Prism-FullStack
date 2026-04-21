# Edge-daemon Makefile.
#
# Toggles:
#   HAS_PIGPIO=1   Compile the PigpioGpioBackend and link -lpigpio. Required on
#                  the Pi; leave unset for host CI builds (falls back to mock).
#   DEBUG=1        -O0 -g instead of -O2.
#
# Standard targets: all (default), clean, install.

CXX      ?= g++
CXXSTD   ?= -std=c++17
WARN     ?= -Wall -Wextra -pedantic
OPT      ?= -O2
ifeq ($(DEBUG),1)
  OPT = -O0 -g
endif

CXXFLAGS ?= $(CXXSTD) $(OPT) $(WARN) -pthread -Iinclude
LDFLAGS  ?= -pthread

TARGET   ?= cliffscanner-edge

# Default: compile everything in src/. On non-Pi hosts, exclude the pigpio
# backend so we don't need the headers installed.
ifeq ($(HAS_PIGPIO),1)
  CXXFLAGS += -DHAS_PIGPIO
  LDLIBS   += -lpigpio -lrt
  SRC := $(wildcard src/*.cpp)
else
  SRC := $(filter-out src/pigpio_gpio_backend.cpp, $(wildcard src/*.cpp))
endif

OBJ := $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJ) $(LDFLAGS) $(LDLIBS)

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

install: $(TARGET)
	install -D -m 0755 $(TARGET) $(DESTDIR)/opt/cliffscanner/bin/$(TARGET)

.PHONY: all clean install
