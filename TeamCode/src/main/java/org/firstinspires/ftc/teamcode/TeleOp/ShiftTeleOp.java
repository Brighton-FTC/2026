package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Shift TeleOp", group = "team2")
public class ShiftTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motor fld = new Motor(hardwareMap,"front_left_drive");
        Motor frd = new Motor(hardwareMap, "front_right_drive");
        Motor rld = new Motor(hardwareMap, "back_left_drive");
        Motor rrd = new Motor(hardwareMap, "back_right_drive");

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()){
            gamepad.readButtons();

            fld.set(gamepad.getLeftY());
            rld.set(gamepad.getLeftY());
            frd.set(gamepad.getRightY());
            rrd.set(gamepad.getRightY());

            sleep(20);
        }
    }
}