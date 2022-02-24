package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.TSEPipeline;

public class FFRobot extends Robot{

    private final LinearOpMode opMode;
    public final LynxModuleSubsystem lynxModuleSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final LiftSubsystem liftSubsystem;
    public final DuckSubsystem duckSubsystem;
    public final CameraSubsystem cameraSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final MecanumDriveSubsystem drive;
    private final Telemetry telemetry;
    public double startTime = 0;

    public FFRobot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isFieldCentric) {
        this.telemetry = telemetry;
        this.opMode = opMode;

        //tells the user that the program has begun initialization
        telemetry.addLine("Robot Initializing");
        telemetry.update();

        //creates all of the subsystems for the robot
        lynxModuleSubsystem = new LynxModuleSubsystem(hardwareMap, telemetry);
        cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry);
        liftSubsystem = new LiftSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, liftSubsystem::intakeAllowed, liftSubsystem::autoExtakeAllowed);
        duckSubsystem = new DuckSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap),telemetry, isFieldCentric);
        odometrySubsystem = new OdometrySubsystem(hardwareMap, telemetry, drive);

        //drive.setLocalizer(FFRobotLocalizer.get(hardwareMap, telemetry));
    }

    public FFRobot(LinearOpMode opMode) {
        this(opMode.hardwareMap, opMode.telemetry,opMode, false);
    }

    public FFRobot(LinearOpMode opMode, boolean isFieldCentric) {
        this(opMode.hardwareMap, opMode.telemetry,opMode, isFieldCentric);
    }

    public void initTele() {
        if (PoseStorage.currentPose.equals(new Pose2d())) {
            PoseStorage.currentPose = new Pose2d(0,0,0);
        }
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
        liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED_TSE_HIGH);

        //tells the user that the program has begun generating the trajectories for autonomous
        telemetry.addLine("Robot Generating Trajectories");
        telemetry.update();
    }

    public void waitForStartAuto() {
        startTime = System.nanoTime() * Math.pow(10,-9);

        while (!opMode.isStarted() || (System.nanoTime() * Math.pow(10,-9) < startTime + 5 && cameraSubsystem.getResult() == null)) {

            if (opMode.isStarted()) {
                startTime = System.nanoTime() * Math.pow(10,-9);
            }

            if (cameraSubsystem.getResult() == null) {
                telemetry.addLine("Robot Vision Initializing");
            } else {
                telemetry.addLine("Robot Ready");
                telemetry.addData("Randomization", cameraSubsystem.getResult());
            }
            telemetry.update();
        }

        //if no detection has occurred after timeout, the Right position is chosen as a default.
        if (cameraSubsystem.getResult() == null) {
            cameraSubsystem.TSEPipeline.lastResult = TSEPipeline.tsePositions.RIGHT;
        }
    }

    public void waitForStartTele() {
        while (!opMode.isStarted()) {
            telemetry.addLine("Robot Ready");
            telemetry.update();
        }
    }

    public void loop() {
        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            run();
        }
    }

    public double timeSinceStart() {
        return System.nanoTime() * Math.pow(10,-9) - startTime;
    }
}
