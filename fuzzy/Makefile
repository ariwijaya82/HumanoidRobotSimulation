COMPILER = g++
SRC = main.cpp
EXE = main

INCLUDE = -I"$(HOME)/fuzzylite-6.0/fuzzylite"

LIBRARIES = -L"$(HOME)/fuzzylite-6.0/fuzzylite/release/bin" -lfuzzylite-static

all:
	$(COMPILER) $(SRC) -o $(EXE) $(INCLUDE) $(LIBRARIES)