#include "why_doesnt_this_work2_new.bak/test_command.h"
#include "why_doesnt_this_work2_new.bak/test_firecontrol.h"
#include "why_doesnt_this_work2_new.bak/test_target_selection.h"

int main() {
    run_command_tests();
    run_firecontrol_tests();
    run_target_selection_tests();
    return 0;
}