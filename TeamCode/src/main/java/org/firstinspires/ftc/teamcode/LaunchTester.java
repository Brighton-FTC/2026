package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LaunchTester extends OpMode {

    private DynamicAngleComponent launcher;

    private final Pose startingPose = new Pose(0, 0, Math.toRadians(0));

    private GamepadEx gamepad;
    private Follower follower;

    private boolean run = false;
    @Override
    public void init() {
        launcher = new DynamicAngleComponent(hardwareMap, "launcherServo", 0, 144, 42, 2, 0.2, startingPose, telemetry);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            run = true;
        }
        else if (gamepad.wasJustPressed(PSButtons.SQUARE)){
            launcher.stop();
            run = false;
        }
        if (run){
            launcher.dynamicMotorPower();
        }


    }
}
