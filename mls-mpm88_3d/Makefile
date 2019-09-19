PLATFORM=$(shell uname)
TARGET=sph

# Find all .c .cc .cpp sources in current directory.
# Will break if you have two files with the same name but different extension. 
CFILES=$(wildcard *.c)
COBJS=$(CFILES:.c=.o) 
CCFILES=$(wildcard *.cc)
CCOBJS=$(CCFILES:.cc=.o) 
CPPFILES=$(wildcard *.cpp)
CPPOBJS=$(CPPFILES:.cpp=.o) 
SOURCES=$(CFILES) $(CCFILES) $(CPPFILES)
OBJS=$(COBJS) $(CCOBJS) $(CPPOBJS)

DEPS=dependencies
MAKEDEP=g++ -MM -MF $(DEPS)

ifeq ($(PLATFORM),Darwin)
# Apple options here
FRAMEWORKS=-framework Accelerate -framework GLUT -framework OpenGL
CFLAGS=-O3
LFLAGS=$(FRAMEWORKS)
else
ifeq ($(PLATFORM),Linux)
# Linux options here
CFLAGS=-O3
LFLAGS=-lglut -lGLU -lGL 
else
# don't know what to do... fill in appropriate commands for your platform here
endif
endif

.PHONY: clean squeakyclean

all: $(TARGET)

clean: 
	$(RM) $(OBJS) $(TARGET)

squeakyclean: clean
	$(RM) $(DEPS)

$(DEPS): $(SOURCES)
	$(MAKEDEP) $(SOURCES)

-include $(DEPS)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(LFLAGS) $(OBJS)

$(COBJS): %.o: %.c
	$(CXX) -c $(CFLAGS) -o $@ $<

$(CCOBJS): %.o: %.cc
	$(CXX) -c $(CFLAGS) -o $@ $<

$(CPPOBJS): %.o: %.cpp
	$(CXX) -c $(CFLAGS) -o $@ $<
