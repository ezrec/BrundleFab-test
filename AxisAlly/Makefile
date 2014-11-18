
CXXFLAGS = -I/usr/include/SDL -I. -Wall -Werror -g3
LDFLAGS = -lSDL -g3

all: simaxis

simaxis.o: simaxis.cpp AxisAlly.h
	$(CXX) $(CXXFLAGS) -c $^

simaxis: simaxis.o
	$(CXX) -o $@ $^ $(LDFLAGS)
