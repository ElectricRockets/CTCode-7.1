package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double HOPPER_CLOSED_POSITION = 0.135;
    public static double HOPPER_OPEN_POSITION = 0.21;
    public static double HOPPER_BARELY_OPEN_POSITION = 0.205;
    public static int LIFT_INTAKE = 0;
    public static int LIFT_LOW = 20;
    public static int LIFT_SHARED = 170;
    public static int LIFT_MID = 300;
    public static int LIFT_HIGH = 605;
    public static double LEFT_ARM_INTAKE = 0.92;
    public static double LEFT_ARM_SCORE = 0.106;
    public static double RIGHT_ARM_INTAKE = 0.03;
    public static double RIGHT_ARM_SCORE = 0.84;
    public static double TSE_ARM_GRAB = 0.9;
    public static double TSE_ARM_STORE_FRONT = 0.2;
    public static double TSE_ARM_STORE_MID = 0.5;
    public static double TSE_ARM_SCORE = 0.7;
    public static double TSE_CLAW_OPEN = 0.5;
    public static double TSE_CLAW_CLOSED = 0.03;

    public static double BASE_DUCK_POWER = 0.1;
    public static double INSIDE_DELIVERY_TIME = 1.5;
    public static double INSIDE_DELIVERY_ACCEL = 0.7;
    public static double INSIDE_DELIVERY_MAX_SPEED = 1;
    public static double OUTSIDE_DELIVERY_TIME = 2;
    public static double OUTSIDE_DELIVERY_ACCEL = 0.3;
    public static double OUTSIDE_DELIVERY_MAX_SPEED = 0.6;
    public static double UNIVERSAL_DELIVERY_TIME = 4;
    public static double UNIVERSAL_DELIVERY_ACCEL = 0.2;
    public static double UNIVERSAL_DELIVERY_MAX_SPEED = 0.3;

    public static double INTAKE_RAMP_LOW = 0.77;
    public static double INTAKE_RAMP_HIGH = 0.91;
    public static double INTAKE_POWER_DEFAULT = 1;
    public static double EXTAKE_POWER_DEFAULT = -0.5;

    public static double ODOMETRY_DEPLOYED = 0;
    public static double ODOMETRY_RETRACTED = 0.5;
    public static double ODOMETRY_DEPLOY_WAIT = 0.5;

    public static double CAMERA_STORED = 1;
    public static double CAMERA_TARGETING = 0.8;

    public static double SLOW_SPEED = 0.3;
    public static double NORMAL_SPEED = 1;

    public static boolean GENERATE_TELEMETRY = true;
    public static boolean GENERATE_SLOWING_TELEMETRY = false;

    public static double WAIT_BETWEEN_MOVEMENTS = 0.5;
    public static double WAIT_AFTER_FREIGHT_REGISTER = 0.2;

    public static int CAMERA_ISO = 200;
    public static int EXPOSURE_MS = 20;
}
