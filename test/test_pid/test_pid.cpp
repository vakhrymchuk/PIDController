#include <unity.h>
#include <Arduino.h>

#include "PIDController.h"

void test_pid_kp(void)
{
    PIDController *pid = new PIDController();
    float P = 1.5;
    pid->tune(P, 0, 0);
    pid->setpoint(100);
    TEST_ASSERT_EQUAL(70 * P, pid->compute(30));
    TEST_ASSERT_EQUAL(70 * P, pid->getP());
    TEST_ASSERT_EQUAL(80 * P, pid->compute(20));
    TEST_ASSERT_EQUAL(-20 * P, pid->compute(120));
    TEST_ASSERT_EQUAL(0, pid->compute(100));
}

void test_pid_kp_ki(void)
{
    PIDController *pid = new PIDController();
    float I = 0.5;
    pid->tune(1, I, 0);
    pid->setpoint(100);

    int time = 10;

    TEST_ASSERT_EQUAL(0, pid->getI());
    TEST_ASSERT_EQUAL(20, pid->compute(80));
    TEST_ASSERT_EQUAL_MESSAGE(0, pid->getI(), "Should be zero because time difference 0");
    delay(time);
    TEST_ASSERT_EQUAL(10 + 10 * I * time, pid->compute(90));
    TEST_ASSERT_EQUAL(10 * I * time, pid->getI());
    delay(time);
    TEST_ASSERT_EQUAL(5 + (10 + 5) * I * time, pid->compute(95));
    TEST_ASSERT_EQUAL(15 * I * time, pid->getI());
}

void test_pid_reset_time(void)
{
    PIDController *pid = new PIDController();
    float I = 0.5;
    pid->tune(1, I, 0);
    pid->setpoint(100);

    int time = 10;

    TEST_ASSERT_EQUAL(20, pid->compute(80));
    TEST_ASSERT_EQUAL(0, pid->getI());
    delay(time);
    pid->resetLastTime();
    TEST_ASSERT_EQUAL(10, pid->compute(90));
    TEST_ASSERT_EQUAL_MESSAGE(0, pid->getI(), "Should not be incremented after reset");
}

void test_pid_kp_kd(void)
{
    PIDController *pid = new PIDController();
    double D = 2;
    pid->tune(1, 0, D);
    pid->setpoint(100);

    int time = 10;
    int value = pid->compute(90);
    delay(time);
    TEST_ASSERT_EQUAL(10, pid->getP());
    TEST_ASSERT_EQUAL_MESSAGE(D * 10 / pid->getTimeChange(), pid->getD(), "Time difference 0");
    TEST_ASSERT_EQUAL(10 + D * 10 / pid->getTimeChange(), value);
    
    delay(time);
    TEST_ASSERT_EQUAL_MESSAGE(10, pid->compute(90), "Only P should be");
    TEST_ASSERT_EQUAL_MESSAGE(0, pid->getD(), "D should be 0 because last_error = error");
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_kp);
    RUN_TEST(test_pid_kp_ki);
    RUN_TEST(test_pid_reset_time);
    RUN_TEST(test_pid_kp_kd);
    UNITY_END();
}

void loop()
{
}
