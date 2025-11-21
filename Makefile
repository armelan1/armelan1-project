INC=include
SRC=src
BUILD=build
APP=app

default:
	g++ ./$(SRC)/*.cc -o ./$(BUILD)/$(APP)

run:
	./$(BUILD)/$(APP)