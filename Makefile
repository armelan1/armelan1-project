INC=include
SRC=src
BUILD=build
APP=main

SRCS=$(wildcard $(SRC)/*.cc)
OBJS=$(SRCS:$(SRC)/%.cc=$(BUILD)/%.o)

$(BUILD)/%.o: $(SRC)/%.cc
	g++ -I$(INC) -c $< -o $@

$(BUILD)/$(APP): $(OBJS)
	g++ $^ -o $@

default: $(BUILD)/$(APP)

run:
	./$(BUILD)/$(APP)
