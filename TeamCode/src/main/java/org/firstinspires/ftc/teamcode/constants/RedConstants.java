package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class RedConstants {
    public static Pose2d CYCLE_START = new Pose2d(16, -59.5, toRadians(-90));
    public static Pose2d CYCLE_DEPOSIT = new Pose2d(-4.75, -34.25, toRadians(120));
    public static Vector2d DEPOSIT_VARIANCE = new Vector2d(2,1);
    public static Pose2d CYCLE_DEPOSIT_REVERSED = new Pose2d(CYCLE_DEPOSIT.vec(), CYCLE_DEPOSIT.getHeading() + toRadians(180));
    public static Pose2d GAP = new Pose2d(25, -64.5, toRadians(0));
    public static Pose2d GAP_INSIDE = new Pose2d(GAP.getX() + 6, GAP.getY(), GAP.getHeading());

    public static Pose2d IW1 = new Pose2d(45, -64.5, toRadians(0));
    public static Pose2d IW2 = new Pose2d(IW1.getX() + 4, IW1.getY(), 0);
    public static Pose2d IW2_OFFSET_X = new Pose2d(6,0,0);
    public static Pose2d IW2_OFFSET_Y = new Pose2d(0,6,0);

    public static Pose2d CYCLE_TSELEFT = new Pose2d(12, -41.5, toRadians(130) + toRadians(180));
    public static Pose2d CYCLE_TSEMID = new Pose2d(12, -46.5, toRadians(90) + toRadians(180));
    public static Pose2d CYCLE_TSERIGHT = new Pose2d(11, -40.5, toRadians(41) + toRadians(180));

    public static Pose2d WAREHOUSE_PARK = new Pose2d(40, -64, toRadians(0));

    public static Pose2d DUCK_START = new Pose2d(-41, -61, toRadians(-90));
    public static Pose2d DUCK_DEPOSIT = new Pose2d(-25.5, -25, toRadians(0));
    public static Pose2d DUCK_DEPOSIT_REVERSED = new Pose2d(DUCK_DEPOSIT.getX(), DUCK_DEPOSIT.getY(), DUCK_DEPOSIT.getHeading() + toRadians(180));
    public static Pose2d DUCK_TSERIGHT = new Pose2d(-36, -44, toRadians(41) + toRadians(180));
    public static Pose2d DUCK_TSEMID = new Pose2d(-36, -48, toRadians(90) + toRadians(180));
    public static Pose2d DUCK_TSELEFT = new Pose2d(-35.5, -43, toRadians(139) + toRadians(180));

    public static Pose2d CAROUSEL = new Pose2d(-61, -58.5, toRadians(-45));
    public static Vector2d DUCKINTAKEEND = new Vector2d(-60,-60);
    public static Pose2d DUCKINTAKESTART = new Pose2d(-40,-60, toRadians(-120+90));

    public static Pose2d DEPOT_PARK = new Pose2d(-60, -36, toRadians(180));
}