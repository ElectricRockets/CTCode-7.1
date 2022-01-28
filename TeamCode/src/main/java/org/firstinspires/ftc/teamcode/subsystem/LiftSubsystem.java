package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

public class LiftSubsystem extends SubsystemBase {

    private final DcMotorEx lift;
    private final Servo leftArm;
    private final Servo rightArm;
    private final Servo hopperClaw;
    private final Servo tseArm;
    private final Servo tseClaw;
    private final ColorRangeSensor hopperColor;
    public final boolean smartIntake;
    public boolean needsToDeregister;
    private long detectionTime = 0;
    private boolean detectingFreight = false;
    private boolean wasDetectingFreight = false;
    private int liftTargetPosition;
    private int liftPosition;
    private final Telemetry telemetry;
    public enum states {
        INTAKE,
        INTAKE_CLOSED,
        INTAKE_CLOSED_TSE_HIGH,
        INTAKE_TSE_HIGH,
        SCORE_LOW_CLOSED,
        SCORE_SHARED_CLOSED,
        SCORE_MID_CLOSED,
        SCORE_HIGH_CLOSED,
        SCORE_LOW_OPEN,
        SCORE_SHARED_OPEN,
        SCORE_MID_OPEN,
        SCORE_HIGH_OPEN,
        GRAB_TSE_OPEN_INTAKE_OPEN,
        GRAB_TSE_CLOSED_INTAKE_OPEN,
        GRAB_TSE_OPEN_INTAKE_CLOSED,
        GRAB_TSE_CLOSED_INTAKE_CLOSED,
        SCORE_TSE_CLOSED_HOPPER_CLOSED,
        SCORE_TSE_OPEN_HOPPER_CLOSED,
        SCORE_TSE_CLOSED_HOPPER_OPEN,
        SCORE_TSE_OPEN_HOPPER_OPEN
    }
    public states state;
    private final PIDFController liftPIDF;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry Telemetry) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        hopperClaw = hardwareMap.get(Servo.class, "hopperClaw");
        tseArm = hardwareMap.get(Servo.class, "tseArm");
        tseClaw = hardwareMap.get(Servo.class, "tseClaw");
        hopperColor = hardwareMap.get(ColorRangeSensor.class, "hopperColor");
        telemetry = Telemetry;
        register();

        state = states.INTAKE;
        smartIntake = true;
        needsToDeregister = false;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftPIDF =  new PIDFController(lift::getCurrentPosition, this::liftTarget, 0.006,0,-0.0002, 1,() -> 0,() -> 0.005,0,-0.65,0.6);
        liftPIDF =  new PIDFController(this::getLiftPosition, this::liftTarget, 0.022,0,-0.01, 15,() -> 0,() -> 0.23,0,-0.38,0.9);
    }

    public void periodic() {

        updateLiftPosition();

        //the lift motor is set to a new power based off of the pidf controller.
        lift.setPower(liftPIDF.getUpdate());

        //code to make sure the TSE arm doesn't get caught on the lift structure
        if (state == states.INTAKE || state == states.INTAKE_CLOSED && tseArm.getPosition() != RobotConstants.TSE_ARM_STORE_FRONT) {
            if (getLiftPosition() > 50) {
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
            } else {
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_FRONT);
            }
        }

        //code to automatically reverse intake when freight is taken in.
        if (smartIntake && ( state == states.INTAKE || state == states.GRAB_TSE_CLOSED_INTAKE_OPEN || state == states.GRAB_TSE_OPEN_INTAKE_OPEN)) {
            //checks to see if freight is detected
            detectingFreight = (hopperColor.getRawLightDetected() > 170);

            //if the freight doesn't need to deregister, either a timer is started, if it is a rising edge, or the lift state is set based off of the current time.
            if (!needsToDeregister) {
                if (!wasDetectingFreight && detectingFreight) {
                    detectionTime = System.nanoTime();
                } else if (detectingFreight && System.nanoTime() - detectionTime > RobotConstants.WAIT_AFTER_FREIGHT_REGISTER * Math.pow(10, 9) && state == states.INTAKE) {
                    setState(states.INTAKE_CLOSED);
                }
                wasDetectingFreight = detectingFreight;
            }

            //if freight needs to deregister, and no freight are registered, it resets that variable to false
            else if (!detectingFreight) {
                needsToDeregister = false;
            }
        }

        //if the robot is set to generate telemetry, telemetry about the robot is generated
        if (RobotConstants.GENERATE_TELEMETRY) {
            telemetry.addData("liftPower",lift.getPower());
            telemetry.addData("liftPosition", liftPosition);
            telemetry.addData("liftTarget", liftTargetPosition);
            if (RobotConstants.GENERATE_SLOWING_TELEMETRY) {
                telemetry.addData("colorSensorLight", hopperColor.getLightDetected());
                telemetry.addData("colorSensorRawLight", hopperColor.getRawLightDetected());
                telemetry.addData("colorSensorDistanceMM", hopperColor.getDistance(DistanceUnit.MM));
            }
        }
    }

    public void setState(states newLiftState) {
        state = newLiftState;
        switch (newLiftState) {
            case INTAKE:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_FRONT);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case INTAKE_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_FRONT);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case INTAKE_TSE_HIGH:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case INTAKE_CLOSED_TSE_HIGH:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_LOW_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_LOW;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_LOW_OPEN:
                liftTargetPosition = RobotConstants.LIFT_LOW;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_SHARED_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_SHARED;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_SHARED_OPEN:
                liftTargetPosition = RobotConstants.LIFT_SHARED;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_MID_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_MID;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_MID_OPEN:
                liftTargetPosition = RobotConstants.LIFT_MID;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_HIGH_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_HIGH_OPEN:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_STORE_MID);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case GRAB_TSE_CLOSED_INTAKE_OPEN:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_GRAB);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case GRAB_TSE_OPEN_INTAKE_OPEN:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_GRAB);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_OPEN);
                break;

            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_GRAB);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case GRAB_TSE_OPEN_INTAKE_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_INTAKE;
                leftArm.setPosition(RobotConstants.LEFT_ARM_INTAKE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_INTAKE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_GRAB);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_OPEN);
                break;

            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_SCORE);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_CLOSED_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_SCORE);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_OPEN);
                break;

            case SCORE_TSE_CLOSED_HOPPER_OPEN:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_SCORE);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_CLOSED);
                break;

            case SCORE_TSE_OPEN_HOPPER_OPEN:
                liftTargetPosition = RobotConstants.LIFT_HIGH;
                leftArm.setPosition(RobotConstants.LEFT_ARM_SCORE);
                rightArm.setPosition(RobotConstants.RIGHT_ARM_SCORE);
                hopperClaw.setPosition(RobotConstants.HOPPER_BARELY_OPEN_POSITION);
                tseArm.setPosition(RobotConstants.TSE_ARM_SCORE);
                tseClaw.setPosition(RobotConstants.TSE_CLAW_OPEN);
                break;
        }
    }

    public boolean intakeAllowed() {
        if (liftPosition > 50) {
            return false;
        }

        switch (state) {
            case INTAKE:
            case INTAKE_TSE_HIGH:
            case GRAB_TSE_OPEN_INTAKE_OPEN:
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
                return true;
            default: return false;
        }
    }

    public boolean autoExtakeAllowed() {
        switch (state) {
            case INTAKE:
            case INTAKE_CLOSED:
            case INTAKE_CLOSED_TSE_HIGH:
            case GRAB_TSE_OPEN_INTAKE_OPEN:
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
            case GRAB_TSE_OPEN_INTAKE_CLOSED:
            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
                return true;
            default: return false;
        }
    }

    public boolean slowDrive() {
        switch (state) {
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
            case GRAB_TSE_OPEN_INTAKE_OPEN:
            case GRAB_TSE_OPEN_INTAKE_CLOSED:
            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
            case SCORE_TSE_CLOSED_HOPPER_OPEN:
            case SCORE_TSE_OPEN_HOPPER_OPEN:
            case SCORE_MID_CLOSED:
            case SCORE_HIGH_CLOSED:
            case SCORE_HIGH_OPEN:
            case SCORE_MID_OPEN:
            case SCORE_SHARED_CLOSED:
            case SCORE_SHARED_OPEN:
                return true;
            default:
                return false;
        }
    }

    private int liftTarget() {
        return liftTargetPosition;
    }

    private void updateLiftPosition() {
        liftPosition = lift.getCurrentPosition();
    }

    private int getLiftPosition() {
        return liftPosition;
    }
}