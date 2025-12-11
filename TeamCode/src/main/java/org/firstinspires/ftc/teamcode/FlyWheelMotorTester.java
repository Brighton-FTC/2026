package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FlyWheelMotorTester extends OpMode {

    private FlyWheelMotorComponent flyWheel;
    private GamepadEx gamepad;

    @Override
    public void init() {
        flyWheel = new FlyWheelMotorComponent(hardwareMap, "flywheel");
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            flyWheel.runMotorAt(1);
        } else if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            flyWheel.stopMotor();
        }
    }
}
