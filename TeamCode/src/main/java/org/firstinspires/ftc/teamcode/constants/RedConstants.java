package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class RedConstants {
    public static Pose2d CYCLE_START = new Pose2d(15, -62, toRadians(-90));
    public static Pose2d CYCLE_DEPOSIT = new Pose2d(-5, -37, toRadians(120));
    public static Vector2d DEPOSIT_VARIANCE = new Vector2d(2,2);
    public static Pose2d CYCLE_DEPOSIT_REVERSED = new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading() + toRadians(180));
    public static Pose2d GAP = new Pose2d(25, -64, toRadians(0));
    public static Pose2d GAP_INSIDE = new Pose2d(GAP.getX() + 6, GAP.getY(), GAP.getHeading());

    public static Pose2d IW1 = new Pose2d(42, -62, toRadians(-35));
    public static Vector2d IW2 = new Vector2d(IW1.getX() + 4, IW1.getY());
    public static Vector2d IW2_OFFSET = new Vector2d(2.5,0);

    public static double LEAVING_HUB_OFFSET = 0.3;
    public static double CYCLE_LIFT_EXTEND_OFFSET = 0;
    public static double CYCLE_INTAKE_START_OFFSET = 6;

    public static Pose2d CYCLE_TSELEFT = new Pose2d(15, -43.5, toRadians(139) + toRadians(180));
    public static Pose2d CYCLE_TSEMID = new Pose2d(13, -47.5, toRadians(90) + toRadians(180));
    public static Pose2d CYCLE_TSERIGHT = new Pose2d(12.5, -43.5, toRadians(41) + toRadians(180));

    public static Pose2d WAREHOUSE_PARK = new Pose2d(40, -64, toRadians(0));

    public static Pose2d DUCK_START = new Pose2d(-29, -62, toRadians(-90));
    public static Pose2d DUCK_DEPOSIT = new Pose2d(-25.5, -25, toRadians(0));
    public static Pose2d DUCK_TSERIGHT = new Pose2d(-36, -44, toRadians(41));
    public static Pose2d DUCK_TSEMID = new Pose2d(-36, -48, toRadians(90));
    public static Pose2d DUCK_TSELEFT = new Pose2d(-36, -44, toRadians(139));

    public static Pose2d CAROUSEL = new Pose2d(-59.5, -57, toRadians(135));
    public static Vector2d DUCKINTAKEEND = new Vector2d(-60,-60);
    public static Pose2d DUCKINTAKESTART = new Pose2d(-45,-60, toRadians(-120));

    public static Pose2d DEPOT_PARK = new Pose2d(-60, -36, toRadians(180));

}