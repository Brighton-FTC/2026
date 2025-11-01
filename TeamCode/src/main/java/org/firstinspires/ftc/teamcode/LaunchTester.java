package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LaunchTester extends OpMode {

    private DynamicAngleComponent launcher;

    private GamepadEx gamepad;

    private Boolean run = false;
    @Override
    public void init() {
        launcher = new DynamicAngleComponent(hardwareMap, "launcherServo", -72, 72, 42, 2, telemetry);
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
