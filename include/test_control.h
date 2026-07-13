#ifndef TEST_CONTROL_H
#define TEST_CONTROL_H

/*
 * Hardware bring-up / bench test mode.
 *
 * This is the counterpart to thermalControl(): instead of running the closed-loop
 * thermal management, it drives each actuator directly from the CAT_TEST parameters
 * so a manufactured unit can be exercised on the bench via the web UI.
 *
 * Only ever called on the `hardware-test` firmware build. thermalControl() is left
 * in the tree but not called there.
 */

void testControl();

#endif // TEST_CONTROL_H
