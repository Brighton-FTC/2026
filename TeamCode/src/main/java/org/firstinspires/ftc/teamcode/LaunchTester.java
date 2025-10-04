package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LaunchTester extends OpMode {

    private DynamicAngleComponent launcher;

    private GamepadEx gamepad;
    @Override
    public void init() {
        launcher = new DynamicAngleComponent(hardwareMap, "launcherServo", -72, 72, 42, 2);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            launcher.dynamicMotorPower();
        }
        else if (gamepad.wasJustPressed(PSButtons.SQUARE)){
            launcher.stop();
        }

    }
}
