package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public interface LocationConstants {
    Pose2d CYCLE_START();
    Pose2d CYCLE_DEPOSIT();
    Vector2d DEPOSIT_VARIANCE();
    Pose2d CYCLE_DEPOSIT_REVERSED();
    Pose2d GAP();
    Pose2d GAP_INSIDE();

    Pose2d IW1();
    Vector2d IW2();
    Vector2d IW2_OFFSET();

    double LEAVING_HUB_OFFSET();
    double CYCLE_LIFT_EXTEND_OFFSET();
    double CYCLE_INTAKE_START_OFFSET();

    Pose2d CYCLE_TSELEFT();
    Pose2d CYCLE_TSEMID();
    Pose2d CYCLE_TSERIGHT();

    Pose2d WAREHOUSE_PARK();

    Pose2d DUCK_START();
    Pose2d DUCK_DEPOSIT();
    Pose2d DUCK_TSERIGHT();
    Pose2d DUCK_TSEMID();
    Pose2d DUCK_TSELEFT();

    Pose2d CAROUSEL();
    Vector2d DUCKINTAKEEND();
    Pose2d DUCKINTAKESTART();

    Pose2d DEPOT_PARK();
}
