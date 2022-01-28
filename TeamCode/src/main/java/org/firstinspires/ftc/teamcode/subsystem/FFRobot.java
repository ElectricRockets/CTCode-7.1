package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.TSEPipeline;

import java.util.function.BooleanSupplier;

public class FFRobot extends Robot{

    public final IntakeSubsystem intakeSubsystem;
    public final LiftSubsystem liftSubsystem;
    public final DuckSubsystem duckSubsystem;
    public final CameraSubsystem cameraSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final MecanumDriveSubsystem drive;
    //public TrackFuser mainLocalizer;
    private final Telemetry telemetry;
    public double startTime = 0;

    public FFRobot(HardwareMap hardwareMap, Telemetry telemetry, boolean isFieldCentric) {
        this.telemetry = telemetry;

        //tells the user that the program has begun initialization
        telemetry.addLine("Robot Initializing");
        telemetry.update();

        //sets lynx modules to do auto bulk reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //creates all of the subsystems for the robot
        cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry);
        liftSubsystem = new LiftSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, liftSubsystem::intakeAllowed, liftSubsystem::autoExtakeAllowed);
        duckSubsystem = new DuckSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap),telemetry, isFieldCentric);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        //drive.setLocalizer(FFRobotLocalizer.get(hardwareMap));
    }

    public FFRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public void initTele() {
        /*if (PoseStorage.currentPose == new Pose2d()) {
            PoseStorage.currentPose = new Pose2d(0,0,0);
        }*/
        drive.setPoseEstimate(PoseStorage.currentPose);
        intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED);
        //odometrySubsystem.setState(OdometrySubsystem.states.RETRACTED);
        liftSubsystem.setState(LiftSubsystem.states.INTAKE);
        cameraSubsystem.setState(CameraSubsystem.states.STORE);

        telemetry.addLine("Robot Ready");
        telemetry.update();
    }

    public void initAuto(Pose2d startPose) {
        drive.setPoseEstimate(startPose);
        intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED);
        //odometrySubsystem.setState(OdometrySubsystem.states.DEPLOYED);
        cameraSubsystem.initCamera();
        liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED);

        //tells the user that the program has begun generating the trajectories for autonomous
        telemetry.addLine("Robot Generating Trajectories");
        telemetry.update();
    }

    public void waitForStart( BooleanSupplier started) {
        startTime = System.nanoTime() * Math.pow(10,-9);

        while (!started.getAsBoolean() || (System.nanoTime() * Math.pow(10,-9) < startTime + 5 && cameraSubsystem.getResult() == null)) {

            if (started.getAsBoolean()) {
                startTime = System.nanoTime() * Math.pow(10,-9);
            }

            if (cameraSubsystem.getResult() == null) {
                telemetry.addLine("Robot Vision Initializing");
                telemetry.update();
            } else {
                telemetry.addLine("Robot Ready");
                telemetry.addData("Randomization", cameraSubsystem.getResult());
                telemetry.update();
            }
        }

        //if no detection has occurred after timeout, the Right position is chosen as a default.
        if (cameraSubsystem.getResult() == null) {
            cameraSubsystem.TSEPipeline.lastResult = TSEPipeline.tsePositions.RIGHT;
        }
    }

    public double timeSinceStart() {
        return System.nanoTime() * Math.pow(10,-9) - startTime;
    }
}
