ifeq ($(UIMACPP_HOME),)
  $(error UIMACPP_HOME not set)
endif

CC := g++
SRCDIR := src
BUILDDIR := bin
EXE_SRC := src/app.cpp

# compile flags
CFLAGS := -Wall -x c++ $(BUILD_CFLAGS)

# link flags
LFLAGS := -L$(UIMACPP_HOME)/lib -luima -lapr-1

# mongo specific link flags
MONGO_LFLAGS := -pthread -lmongoclient -lboost_thread \
	 -lboost_system -lboost_regex

ifeq ($(DEBUG), 1) # debugging
  CFLAGS += -DDEBUG -g -fno-inline -fno-default-inline -fPIC
else
  LFLAGS += -Wl,--strip-debug
  CFLAGS += -DNDEBUG -DTRACEOFF -O3 -fPIC
  # suppress some uima specific warnings
  CFLAGS += -Wno-unused-but-set-variable -Wno-deprecated
endif

# include directories for compile
INCLUDES := -I$(UIMACPP_HOME)/include -I$(UIMACPP_HOME)/include/apr-1

# executable specific linkflags
EXE_LFLAGS := -lxerces-c -licuuc -licuio -licui18n -licudata \
	 -ldl -Wno-deprecated


SRCS := $(shell find $(SRCDIR) -maxdepth 1 -type f \
	 -name *Populator.cpp -o -name *Annotator.cpp)
LIBS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SRCS:.cpp=.so))
DEPS_SRCS := src/StdCoutLogger.cpp
DEPS_OBJS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(DEPS_SRCS:.cpp=.o))
EXE := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(EXE_SRC:.cpp=))

# add mongo linkflags just for specific annotators
bin/JointStatePopulator.so: LFLAGS+=$(MONGO_LFLAGS)

# build shared libraries and executable
all: $(LIBS) $(EXE)

# build executable
$(EXE): $(DEPS_OBJS) $(EXE_SRC)
	$(CC) $(DEPS_OBJS) $(EXE_SRC) \
	 $(MONGO_LFLAGS) \
	 $(LFLAGS) \
	 $(INCLUDES) \
	 $(EXE_LFLAGS) -g \
	 -o $(EXE) $(INC) $(LIB)

# build specific shared library, *.so
$(BUILDDIR)/%.so: $(BUILDDIR)/%.o $(DEPS_OBJS)
	$(CC) $(DEPS_OBJS) $< \
	$(LFLAGS) -shared \
	-o $@

# build specific object, *.o
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) \
	 $(INCLUDES) \
	 -c $< -o $@

# remove ./bin directory
clean:
	rm -r $(BUILDDIR)

# build all dependencies and run executable
run: all
	$(EXE)
