
DBGFLAGS=
ifeq ($(DEBUG),on)
DBGFLAGS=-DDEBUG
endif
CPPFLAGS=-I../src/libraries $(DBGFLAGS)
CXXFLAGS=-g -std=c++11
CFLAGS=-g

GCC=gcc $(CFLAGS) $(CPPFLAGS)
GXX=g++ $(CXXFLAGS) $(CPPFLAGS)
LD=ld $(LDFLAGS)

LIB_SRC=\
	 ../src/libraries/Debug/Debug.cpp \
	 ../src/libraries/Config/Config.cpp

LIB_OBJ=$(LIB_SRC:.cpp=.o)

LIB_CPP=$(LIB_SRC:.cpp=.d)

TASKS=$(LIB_CPP) param.d debug.d config.d debug config

.SUFFIXES: .c .cpp .d

all: $(TASKS)

debug: debug.o $(LIB_OBJ)
	$(GXX) -o $@ $^

config: config.o $(LIB_OBJ)
	$(GXX) -o $@ $^

clean::
	@-rm $(LIB_OBJ) $(TASKS) *.o *.d *~ >/dev/null 2>/dev/null || true

.c.d:
	$(GCC) -E $< > $@

.cpp.d:
	$(GXX) -E $< > $@

.cpp.o:
	$(GXX) -c $< -o $@
