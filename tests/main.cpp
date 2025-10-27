#include "tests/test_approximate_math.h"
#include "tests/test_command.h"
#include "tests/test_firecontrol.h"
#include "tests/test_serializer.h"
#include "tests/test_target_selection.h"
#include "tests/test_vector.h"

int main() {
	run_approximate_math_tests();
	run_command_tests();
	run_firecontrol_tests();
	run_serializer_tests();
	run_target_selection_tests();
	run_vector_tests();
	return 0;
}