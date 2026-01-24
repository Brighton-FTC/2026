package org.firstinspires.ftc.teamcode.AprilTag;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PSButtons;


@TeleOp
public class AprilTagTester extends OpMode {

    GamepadEx gamepad;

    private boolean webcamOn = false;

    private int state;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    @Override
    public void init(){
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);

        gamepad = new GamepadEx(gamepad1);

        camera.stopStreaming();

    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(PSButtons.SQUARE) && !webcamOn){
            state = 1;
            webcamOn = true;
        } else if (gamepad.wasJustPressed(PSButtons.SQUARE) && webcamOn) {
            camera.stopStreaming();
            webcamOn = false;
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE) && !webcamOn){
            state = 2;
            webcamOn = true;
        } else if (gamepad.wasJustPressed(PSButtons.CIRCLE) && webcamOn) {
            state = 3;
            camera.stopStreaming();
            webcamOn = false;
        }
        if (state == 1){
            camera.startStreaming();
            camera.telemetryAprilTag();

        } else if (state == 2) {
            camera.startStreaming();
            camera.checkCase();
        }


        telemetry.update();
    }

}
