CC=g++ -fopenmp
FLAGS=-std=c++17 -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG


all: NetworkDebug NetworkRelease UltraTripBasedDebug UltraTripBasedRelease

NetworkDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Network Network.cpp

NetworkRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Network Network.cpp

UltraTripBasedDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o UltraTripBased UltraTripBased.cpp

UltraTripBasedRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o UltraTripBased UltraTripBased.cpp

