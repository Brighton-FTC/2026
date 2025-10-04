package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class AprilTagTester extends OpMode {

    GamepadEx gamepad;

    private boolean webcamOn = false;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    @Override
    public void init(){
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1");

        gamepad = new GamepadEx(gamepad1);

    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(PSButtons.SQUARE) && !webcamOn){
            camera.startStreaming();
            camera.telemetryAprilTag();
        } else if (gamepad.wasJustPressed(PSButtons.SQUARE) && webcamOn) {
            camera.stopStreaming();
            webcamOn = false;
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE) && !webcamOn){
            camera.startStreaming();
            camera.checkCase();
        } else if (gamepad.wasJustPressed(PSButtons.CIRCLE) && webcamOn) {
            camera.stopStreaming();
            webcamOn = false;
        }
    }

}
