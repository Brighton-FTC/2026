package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "William's Team Drivetrain", group = "team2")
public class Team2Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        motors[1].setInverted(true);
        motors[3].setInverted(true);


        MecanumDrive drive = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        imu.resetYaw();

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                motors[0].setInverted(!motors[0].getInverted());
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                motors[1].setInverted(!motors[1].getInverted());
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                motors[2].setInverted(!motors[2].getInverted());
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                motors[3].setInverted(!motors[3].getInverted());
            }

            drive.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            telemetry.addLine("Robot Centric");

            telemetry.addData("FL", motors[0].getInverted());
            telemetry.addData("FR", motors[1].getInverted());
            telemetry.addData("BL", motors[2].getInverted());
            telemetry.addData("BR", motors[3].getInverted());

            telemetry.update();
            sleep(20);
        }
    }
}
