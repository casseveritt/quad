
UNAME := $(shell uname)
$(info UNAME is $(UNAME))

WARNFLAGS := -Wall -Wextra -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-declarations -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow  -Wsign-promo -Wstrict-overflow=5 -Wswitch-default -Wundef

CFLAGS_NOWARN := -g -std=c++11
CFLAGS := $(CFLAGS_NOWARN) #$(WARNFLAGS)


all: quad

H := $(wildcard *.h)

SRC := quad.cpp

OBJ := $(patsubst %.cpp, %.o, $(SRC))

.cpp.o:
	g++ $(CFLAGS) $< -c

quad: $(OBJ) $(H)
	g++ $(CFLAGS) $(OBJ) -o quad 

format: format_h format_cpp

format_h: $(H)
	for h in $^; do clang-format -style=file -i $$h; done

format_cpp: $(SRC)
	for cpp in $^; do clang-format -style=file -i $$cpp; done

clean:
	rm -f *.o
	rm -f gl
	rm -f *~
	rm -f .*~


