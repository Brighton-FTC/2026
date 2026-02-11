package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.TankDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank drivetrain")
public class TankDriveTrain extends LinearOpMode {


    private Motor left_front_1, left_front_2, left_back_1, left_back_2;
    private Motor right_front_1, right_front_2, right_back_1, right_back_2;

    private MotorGroup leftSide, rightSide;
    private TankDrive drive;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() {
        left_front_1 = new Motor(hardwareMap, "left_front_1");
        left_front_2 = new Motor(hardwareMap, "left_front_2");
        left_back_1 = new Motor(hardwareMap, "left_back_1");
        left_back_2 = new Motor(hardwareMap, "left_back_2");
        right_front_1 = new Motor(hardwareMap, "right_front_1");
        right_front_2 = new Motor(hardwareMap, "right_front_2");
        right_back_1 = new Motor(hardwareMap, "right_back_1");
        right_back_2 = new Motor(hardwareMap, "right_back_2");

        leftSide = new MotorGroup(left_front_1, left_front_2, left_back_1, left_back_2);
        rightSide = new MotorGroup(right_front_1, right_front_2, right_back_1, right_back_2);

        drive = new TankDrive(leftSide, rightSide);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            drive.tankDrive(gamepad.getLeftY(), gamepad.getRightY());
        }
    }
}