package org.firstinspires.ftc.teamcode.FlyWheel;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

@TeleOp
public class FlyWheelMotorTester extends OpMode {

    private FlyWheelMotorPIDComponent flyWheel;
    private GamepadEx gamepad;

    @Override
    public void init() {
        flyWheel = new FlyWheelMotorPIDComponent(hardwareMap, "turretMotor");
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
        if (gamepad.wasJustPressed(PSButtons.CROSS)){
            flyWheel.runMotorAt(0.5);
        }
        telemetry.addData("vel", flyWheel.getVel());
        telemetry.update();
    }
}
