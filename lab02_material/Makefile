# Note this assumes that PKG_CONFIG_PATH is setup correctly
PLAYER_LIB=`pkg-config --cflags --libs playerc++`

common= common_functions.cc \
		cmdline_parsing.cc

bin = lab2_part1 lab2_part2

all: $(bin)

%: %.cc $(common)
	echo Player: $(PLAYER_LIB)
	g++ -Wall -o $@ $(PLAYER_LIB)   $^ 

clean:
	rm -f $(bin)
