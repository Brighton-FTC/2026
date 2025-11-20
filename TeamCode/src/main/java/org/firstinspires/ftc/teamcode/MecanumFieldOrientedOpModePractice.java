package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivePractice;


@TeleOp
public class MecanumFieldOrientedOpModePractice extends OpMode {
    MecanumDrivePractice drive = new MecanumDrivePractice();
    double forward,strafe,rotate;
    private final double DEAD_ZONE_SIZE = 0.2;


    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE_SIZE ? gamepad1.left_stick_y : 0;
        strafe = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE_SIZE ? gamepad1.left_stick_x : 0;
        rotate = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE_SIZE ? gamepad1.right_stick_x : 0;

        drive.driveFieldRelative(forward,strafe,rotate);

    }
}