/* 复用现有模拟时钟、轴驱动和传感器夹具，不改变原回归入口。 */
#define main robot_arm_original_test_main
#include "robot_arm_logic_test.c"
#undef main

/** 验证同步完整移动、并发补充找零以及 STOP、超时、驱动异常边界。 */
int main(void)
{
    uint32_t starts;
    RobotArmStatus_t status;
    /* 0x01 是唯一允许未知坐标直接找零的 MOVE_TO 模式；0x00 仍不得绕过坐标保护。 */
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0, 0, 0, 800u, 600u, 400u,
        ROBOT_MOVE_MOTION_SEQUENTIAL) == ROBOT_ARM_ERR_POSITION_UNKNOWN);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0, 0, 0, 800u, 600u, 400u,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); RobotArm_Task();
    TEST_CHECK(s_busy[ROBOT_AXIS_X] && s_busy[ROBOT_AXIS_Y] &&
               s_busy[ROBOT_AXIS_Z]);
    TEST_CHECK(s_remaining[ROBOT_AXIS_X] == 5000u &&
               s_remaining[ROBOT_AXIS_Y] == 5000u &&
               s_remaining[ROBOT_AXIS_Z] == 5000u);
    TestSensorSnapshot(1u, 1u, 1u);
    RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && RobotArm_IsHomed(ROBOT_AXIS_X) &&
               RobotArm_IsHomed(ROBOT_AXIS_Y) && RobotArm_IsHomed(ROBOT_AXIS_Z));
    /* 0x01 三轴并发但各自保持请求速度；距离短的 Y 轴不得被降速。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 0);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(1000, 100, 1000,
        10000u, 10000u, 100u, ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_X] == 10000u &&
               s_last_start_speed[ROBOT_AXIS_Y] == 10000u &&
               s_last_start_speed[ROBOT_AXIS_Z] == 100u);
    RobotArm_Stop();
    TestResetAndHomeAll();
    TestSetPose(20000, 15000, 30000);
    /* 提前触发 Home 不能截断 20000 个负向脉冲，也不能提前提交坐标。 */
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,15000,30000,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); starts = s_start_count[0];
    TEST_CHECK(s_direction[0] == -1 && s_remaining[0] == 20000u);
    TestSensorSnapshot(1,0,0);
    TEST_CHECK(s_busy[0] && RobotArm_GetX() == 20000);
    TEST_CHECK(!RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_X));
    TestDriverComplete(ROBOT_AXIS_X); RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && RobotArm_GetX() == 0);
    TEST_CHECK(s_start_count[0] == starts);
    TestSensorSnapshot(0,0,0); TestSetPose(20000,15000,30000);
    /* X 先到位必须等 Y 完成，然后 X/Y 同时搜索；Z 不再移动。 */
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,0,30000,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_remaining[0] == 20000u && s_remaining[1] == 15000u);
    starts = s_start_count[0];
    TestDriverComplete(ROBOT_AXIS_X); RobotArm_Task();
    TEST_CHECK(s_start_count[0] == starts && RobotArm_IsBusy());
    TestDriverComplete(ROBOT_AXIS_Y); RobotArm_Task();
    TEST_CHECK(s_busy[0] && s_busy[1] && !s_busy[2]);
    /* 补零开始后，page0 使用的整体状态与子状态必须同时表示仍在执行。 */
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.arm_state == ROBOT_ARM_MOVING && status.operation == ROBOT_OP_MOVE_TO);
    TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_XYZ_WAIT);
    TEST_CHECK(s_remaining[0] == 5000u && s_remaining[1] == 5000u);
    TEST_CHECK(s_last_start_speed[0] == 800u && s_last_start_speed[1] == 600u);
    starts = s_start_count[0];
    RobotArm_Task(); RobotArm_GetStatus(&status); RobotArm_Task();
    TEST_CHECK(s_start_count[0] == starts && RobotArm_IsBusy());
    TestDriverComplete(ROBOT_AXIS_X); RobotArm_Task();
    TEST_CHECK(s_start_count[0] == starts + 1u && s_remaining[0] == 5000u);
    TestSensorSnapshot(1,0,0);
    TEST_CHECK(!s_busy[0] && s_busy[1] && RobotArm_GetX() == 0);
    TestSensorSnapshot(0,0,0); RobotArm_Task();
    TEST_CHECK(!s_busy[0] && RobotArm_IsBusy());
    /* 一轴补零完成不能把仍有其他轴搜索的请求提前标为 DONE。 */
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_XYZ_WAIT);
    TestSensorSnapshot(0,1,0); RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && RobotArm_GetY() == 0);
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.arm_state == ROBOT_ARM_IDLE && status.operation == ROBOT_OP_NONE);
    TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_IDLE);
    TEST_CHECK(status.error_code == 0 && status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED);
    /* 非零目标只执行普通位移，绝不追加搜索。 */
    TestSensorSnapshot(0,0,0); TestSetPose(20000,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(5000,100,100,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); starts = s_start_count[0];
    TEST_CHECK(s_remaining[0] == 15000u);
    TestDriverComplete(ROBOT_AXIS_X); RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && s_start_count[0] == starts);
    /* 零距离也要检查 Home；STOP 后不得重新启动。 */
    TestSetPose(0,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,100,100,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); RobotArm_Task();
    TEST_CHECK(s_busy[0] && s_remaining[0] == 5000u);
    RobotArm_Stop(); starts = s_start_count[0];
    RobotArm_Task(); TestSensorSnapshot(0,0,0); RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && !s_busy[0] && s_start_count[0] == starts);
    /* 超时覆盖普通移动和搜索，并停止其他搜索轴。 */
    TestResetAndHomeAll(); TestSetPose(100,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,0,100,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); s_now_ms += 600u;
    TestDriverComplete(ROBOT_AXIS_X); TestDriverComplete(ROBOT_AXIS_Y); RobotArm_Task();
    s_now_ms += 401u; RobotArm_Task(); RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_HOME_TIMEOUT && !s_busy[0] && !s_busy[1]);
    /* 驱动未真实进入 Busy 必须失败，不能假报搜索成功。 */
    TestResetAndHomeAll(); TestSetPose(100,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,100,100,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); TestDriverComplete(ROBOT_AXIS_X);
    s_start_enters_busy[0] = 0u; RobotArm_Task(); RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_DRIVER && !RobotArm_IsBusy());
    s_start_enters_busy[0] = 1u;
    /* Z 同样只在普通位移后搜索，S3 命中时立即停轴建立零点。 */
    TestResetAndHomeAll(); TestSetPose(100,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(100,100,0,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_direction[2] == -1 && s_remaining[2] == 100u);
    TestDriverComplete(ROBOT_AXIS_Z); RobotArm_Task();
    TEST_CHECK(s_busy[2] && s_remaining[2] == 5000u);
    TestSensorSnapshot(0,0,1);
    TEST_CHECK(!s_busy[2] && RobotArm_GetZ() == 0);
    RobotArm_Task(); TEST_CHECK(!RobotArm_IsBusy());
    /* 第一阶段 STOP 清除延后标志，后续任务轮询不能启动补充搜索。 */
    TestSensorSnapshot(0,0,0); TestSetPose(100,100,100);
    TEST_CHECK(RobotArm_MoveToWithSpeedAndMode(0,100,100,800,600,400,
        ROBOT_MOVE_MOTION_XYZ_SYNC) == ROBOT_ARM_OK);
    RobotArm_Task(); RobotArm_Stop(); starts = s_start_count[0];
    TEST_CHECK(!RobotArm_ShouldDeferHome(ROBOT_AXIS_X));
    RobotArm_Task(); TEST_CHECK(!s_busy[0] && s_start_count[0] == starts);
    /* 0x00 仍保留途中 Home 命中即完成的原有语义，不等待完整步数。 */
    TestResetAndHomeAll(); TestSetPose(100,100,100);
    TEST_CHECK(RobotArm_MoveTo(0,100,100) == ROBOT_ARM_OK);
    RobotArm_Task(); starts = s_start_count[0];
    TestSensorSnapshot(1,0,0);
    TEST_CHECK(!s_busy[0] && RobotArm_GetX() == 0);
    RobotArm_Task(); RobotArm_Task(); RobotArm_Task();
    TEST_CHECK(!RobotArm_IsBusy() && s_start_count[0] == starts);
    return s_test_failure;
}
