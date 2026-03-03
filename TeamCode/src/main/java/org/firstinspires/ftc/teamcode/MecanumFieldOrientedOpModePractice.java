package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivePractice;


@TeleOp
public class MecanumFieldOrientedOpModePractice extends LinearOpMode {
    MecanumDrivePractice drive = new MecanumDrivePractice();
    double forward,strafe,rotate;
    private final double DEAD_ZONE_SIZE = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        forward = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE_SIZE ? gamepad1.left_stick_y : 0;
        strafe = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE_SIZE ? gamepad1.left_stick_x : 0;
        rotate = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE_SIZE ? gamepad1.right_stick_x : 0;
        forward = (Math.pow(forward, 3) + forward) / 2;
        strafe = (Math.pow(strafe, 3) + strafe) / 2;
        rotate = (Math.pow(rotate, 3) + rotate) / 2;

        drive.drive(forward,strafe,rotate);
    }
}