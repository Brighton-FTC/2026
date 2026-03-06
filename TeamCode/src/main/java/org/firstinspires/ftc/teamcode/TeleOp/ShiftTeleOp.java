package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Shift TeleOp", group = "team2")
public class ShiftTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motor fld = new Motor(hardwareMap,"front_left_drive");
        Motor frd = new Motor(hardwareMap, "front_right_drive");
        Motor arm = new Motor(hardwareMap, "arm");

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()){
            gamepad.readButtons();

            fld.set(gamepad.getLeftY());
            frd.set(gamepad.getRightY());
            if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm.set(0.25);
            }
            if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm.set(0);
            }
            if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                arm.set(-0.25);
            }
            if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                arm.set(0);
            }

            sleep(20);
        }
    }
}