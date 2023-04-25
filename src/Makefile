# Compiler settings
CXX = g++
CXXFLAGS = -Wall -std=c++14 -fconcepts-ts
LDFLAGS = -lwiringPi

# Directories
SRCDIR = cpp
LOGSDIR = logs
COMMUNDIR = $(SRCDIR)/Commun
LOGGERDIR = $(COMMUNDIR)/Logger
COORDINATESDIR = $(SRCDIR)/Coordinates
MOTORSDIR = $(SRCDIR)/Motors
JACKDIR = $(SRCDIR)/Jack
ENCODERWHEELSDIR = $(SRCDIR)/EncoderWheels
INCLUDEDIRS = -I$(COMMUNDIR) -I$(MOTORSDIR) -I$(ENCODERWHEELSDIR) -I$(LOGGERDIR) -I$(COORDINATESDIR) -I$(JACKDIR)

# Source and object files
SRC = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(COMMUNDIR)/*.cpp) $(wildcard $(MOTORSDIR)/*.cpp) $(wildcard $(ENCODERWHEELSDIR)/*.cpp) $(wildcard $(LOGGERDIR)/*.cpp) $(wildcard $(COORDINATESDIR)/*.cpp) $(wildcard $(JACKDIR)/*.cpp)
OBJ = $(SRC:.cpp=.o)
LOGS = $(wildcard $(LOGSDIR)/*.log)

# Output
EXEC = main

# Rules
all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) -o $(EXEC)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDEDIRS) -c $< -o $@

clean:
	rm -f $(OBJ) $(EXEC)
	rm -f $(LOGS)

.PHONY: all clean
