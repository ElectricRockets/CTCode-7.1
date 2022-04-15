package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.vision.TSEPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class CameraSubsystem extends SubsystemBase {
    private final HardwareMap hardwareMap;

    private final Servo cameraServo;
    public Vector2d rightCameraPosition = new Vector2d(-5.23, 3.83);

    public final TSEPipeline TSEPipeline;
    public enum states {STORE, TARGETING_BLUE, TARGETING_RED}
    private states state;
    public OpenCvWebcam camera;
    private Telemetry telemetry;

    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        cameraServo = hardwareMap.get(Servo.class, "rightCamera");
        TSEPipeline = new TSEPipeline(telemetry);
    }

    public void initCamera() {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam1")
        );

        camera.setPipeline(TSEPipeline);
        camera.setMillisecondsPermissionTimeout(10000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                startStreaming();
                //camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                //camera.getExposureControl().setExposure(RobotConstants.EXPOSURE_MS, TimeUnit.MILLISECONDS);
                //camera.getGainControl().setGain(RobotConstants.CAMERA_ISO);

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        setState(states.TARGETING_BLUE);
    }

    public void stopStreaming() {
        if (camera != null) {
            camera.stopStreaming();
        }
    }

    public void startStreaming() {
        if (camera != null) {
            camera.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
        }
    }

    public void closeCamera() {
        if (camera != null) {
            camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {

                }
            });
        }
    }

    public TSEPipeline.tsePositions getResult() {
        return TSEPipeline.lastResult;
    }

    public void setState(states newState) {
        state = newState;
        switch (state) {
            case STORE:
                cameraServo.setPosition(RobotConstants.CAMERA_STORED);
                break;
            case TARGETING_BLUE:
                cameraServo.setPosition(RobotConstants.CAMERA_TARGETING_BLUE);
                break;
            case TARGETING_RED:
                cameraServo.setPosition(RobotConstants.CAMERA_TARGETING_RED);
                break;
        }
    }
}
