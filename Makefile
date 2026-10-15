# Host-side unit tests for the portable core (no Arduino needed).
SRC = firmware/shipguard
test:
	g++ -std=c++11 -Wall -Wextra -iquote $(SRC) firmware/tests/test_core.cpp \
	    $(SRC)/crc8.cpp $(SRC)/frame.cpp $(SRC)/features.cpp $(SRC)/risk.cpp $(SRC)/fsm.cpp \
	    -o firmware/tests/test_core && ./firmware/tests/test_core
