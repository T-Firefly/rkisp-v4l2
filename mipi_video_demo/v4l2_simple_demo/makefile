INCLUDE = $(shell pkg-config --cflag opencv)

LIBS = $(shell pkg-config --libs opencv)

BIN = opencv

OBJECTS = firefly_v4l2_simple_demo.o

SOURCES = firefly_v4l2_simple_demo.cpp


$(BIN):$(OBJECTS)
	g++ -o $(BIN) $(OBJECTS) -I $(INCLUDE) $(LIBS)

$(OBJECTS):$(SOURCES)
	g++ -c $(SOURCES)

clean:
	rm $(OBJECT) $(BIN)
