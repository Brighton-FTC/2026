package org.firstinspires.ftc.teamcode.FlyWheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

@TeleOp
public class FlyWheelMotorTester extends OpMode {

    private FlyWheelMotorPIDComponent flyWheel;
    private GamepadEx gamepad;

    private double setpoint;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flyWheel = new FlyWheelMotorPIDComponent(hardwareMap, "flyWheelMotor");
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {

        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            setpoint = 2000;
        } else if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            flyWheel.stopMotor();
            setpoint = 0;
        }
        if (gamepad.wasJustPressed(PSButtons.CROSS)){
            setpoint = 1000;
        }

        if (setpoint > 0) {
            flyWheel.runMotorAt(setpoint);
        }
        telemetry.addData("power", flyWheel.getPower());
        telemetry.addData("vel", flyWheel.getVel());
        telemetry.update();
    }
}
