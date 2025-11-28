INC    = include
SRC    = src
TEST   = tests
EXTERN = extern/UnitTest++
BUILD  = build
APP    = main

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra
COVERAGE_FLAGS = --coverage -fprofile-arcs -ftest-coverage
APP_CXXFLAGS  = $(CXXFLAGS) -I$(INC)
TEST_CXXFLAGS = $(CXXFLAGS) -I$(INC) -I$(EXTERN)


DIRS = $(BUILD) $(BUILD)/tests $(BUILD)/unittest $(BUILD)/unittest/Posix
$(shell mkdir -p $(DIRS))

SRCS = $(wildcard $(SRC)/*.cc)
OBJS = $(SRCS:$(SRC)/%.cc=$(BUILD)/%.o)

$(BUILD)/%.o: $(SRC)/%.cc
	$(CXX) $(APP_CXXFLAGS) -c $< -o $@

$(BUILD)/$(APP): $(OBJS)
	$(CXX) $^ -o $@

default: $(BUILD)/$(APP)

run:
	./$(BUILD)/$(APP)

UNITTEST_SRC  = $(wildcard $(EXTERN)/UnitTest++/*.cpp) $(wildcard $(EXTERN)/UnitTest++/Posix/*.cpp)
UNITTEST_OBJS = $(patsubst $(EXTERN)/UnitTest++/%.cpp,$(BUILD)/unittest/%.o,$(UNITTEST_SRC))

$(BUILD)/unittest/%.o: $(EXTERN)/UnitTest++/%.cpp
	$(CXX) -I$(EXTERN) $(CXXFLAGS) -c $< -o $@

TEST_SRCS = $(wildcard $(TEST)/*.cc)
TEST_OBJS = $(TEST_SRCS:$(TEST)/%.cc=$(BUILD)/tests/%.o)

$(BUILD)/tests/%.o: $(TEST)/%.cc
	$(CXX) $(TEST_CXXFLAGS) -c $< -o $@

$(BUILD)/run_tests: $(filter-out $(BUILD)/main.o,$(OBJS)) $(UNITTEST_OBJS) $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(BUILD)/run_tests
	./$(BUILD)/run_tests

clean:
	rm -f $(BUILD)/*.o $(BUILD)/$(APP) $(BUILD)/run_tests
	rm -rf $(BUILD)/tests

# Coverage targets
coverage:
	@$(MAKE) clean
	@mkdir -p $(DIRS)
	@$(MAKE) test CXXFLAGS="$(CXXFLAGS) $(COVERAGE_FLAGS)"
	lcov --capture --directory $(BUILD) --output-file $(BUILD)/coverage.info --ignore-errors mismatch
	lcov --remove $(BUILD)/coverage.info '/usr/*' '*/extern/*' '*/tests/*' --output-file $(BUILD)/coverage.info
	genhtml $(BUILD)/coverage.info --output-directory $(BUILD)/coverage_report
	@echo "Coverage report generated in $(BUILD)/coverage_report/index.html"

coverage-clean:
	rm -f $(BUILD)/*.gcda $(BUILD)/*.gcno $(BUILD)/*.gcov
	rm -f $(BUILD)/coverage.info
	rm -rf $(BUILD)/coverage_report
