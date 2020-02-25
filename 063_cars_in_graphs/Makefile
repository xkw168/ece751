CFLAGS= -Wall -Werror -std=gnu++11 -pedantic -ggdb3

start:  studentinterface.hpp
	g++ $(CFLAGS)  studentinterface.cpp -o compete-st -DST -L . -lcars

clean:
	rm -f compete-st *~
