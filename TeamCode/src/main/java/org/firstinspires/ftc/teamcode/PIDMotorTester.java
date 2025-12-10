package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class PIDMotorTester extends OpMode {

    private PIDMotorComponent motor;
    private GamepadEx gamepad;

    @Override
    public void init() {
        motor = new PIDMotorComponent(hardwareMap, "turretMotor", telemetry);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        telemetry.update();
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            motor.startVelControlTest();
        }
        else if (gamepad.wasJustPressed(PSButtons.TRIANGLE)){
            motor.VelControlTest1();
        }
        else if (gamepad.wasJustPressed(PSButtons.CROSS)){
            motor.VelControlTest2();
        }
        else if (gamepad.wasJustPressed(PSButtons.SQUARE)){
            motor.startPosControlTest();
        }
    }
}
