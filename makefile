
ifeq ($(UIMACPP_HOME),)
  $(error UIMACPP_HOME not set)
endif

EXTRA_LINK_LIBS=-lapr-1

ifeq ($(DEBUG), 1)
  # compile flags for debug mode
  BUILD_CFLAGS := -DDEBUG -g -fno-inline -fno-default-inline -fPIC
else
  # compile flags for ship mode:
  # all optimization on, Multithreaded, dynamic link runtime
  # suppress unused-but-set-variable warning which results from UIMA, yay.
  BUILD_CFLAGS := -DNDEBUG -DTRACEOFF -O3 -fPIC -Wno-unused-but-set-variable -Wno-unused-function
  BUILD_LFLAGS := -Wl,--strip-debug
endif

# include directory for compile
INCLUDES := -I$(UIMACPP_HOME)/include -I$(UIMACPP_HOME)/include/apr-1

# compiler flags:
CFLAGS := -Wall -x c++ $(BUILD_CFLAGS) -Wno-deprecated

MONGO_LINKFLAGS := -pthread -lmongoclient -lboost_thread \
	 -lboost_system -lboost_regex

BIN_LINKFLAGS := -lxerces-c -licuuc -licuio -licui18n -licudata -ldl

LINKFLAGS := $(BUILD_LFLAGS) -L$(UIMACPP_HOME)/lib -luima $(EXTRA_LINK_LIBS)

CC := g++
SRCDIR := src
BUILDDIR := bin

SRCS := $(shell find $(SRCDIR) -maxdepth 1 -type f \
	 -name *Populator.cpp -o -name *Annotator.cpp)
LIBS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SRCS:.cpp=.so))
DEPS_SRCS := bin/utils.cpp
DEPS_OBJS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(DEPS_SRCS:.cpp=.o))
EXE_SRC := src/app.cpp
EXE     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(EXE_SRC:.cpp=))

# build shared libraries and executable
all: $(DEPS_OBJS) $(LIBS)
	$(CC) $(DEPS_OBJS) $(EXE_SRC) \
	 $(MONGO_LINKFLAGS) $(LINKFLAGS) $(INCLUDES) $(BIN_LINKFLAGS) \
	 -o $(EXE) $(INC) $(LIB)

# build specific object, *.o
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) \
	 $(INCLUDES) \
	 -c $< -o $@

# build specific shared library, *.so
$(BUILDDIR)/%.so: $(BUILDDIR)/%.o $(DEPS_OBJS)
	$(CC) $(DEPS_OBJS) $< \
	 $(MONGO_LINKFLAGS) \
	 $(LINKFLAGS) -shared \
	 -o $@

# remove ./bin directory
clean:
	rm -r $(BUILDDIR)

# run executable
run:
	$(EXE)
