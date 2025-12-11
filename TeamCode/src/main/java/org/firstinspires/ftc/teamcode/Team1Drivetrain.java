package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Eason's Team Drivetrain", group = "team1")
public class Team1Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean fieldCentric = false;
        double maxPower = 0.9;

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };


        MecanumDrive drive = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();
            if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
                fieldCentric = !fieldCentric;
            }

            if (fieldCentric) {
                double yaw = imu.getRobotYawPitchRollAngles().getYaw();
                drive.driveFieldCentric((gamepad1.left_stick_y*maxPower), -(gamepad1.left_stick_x*maxPower), (gamepad1.right_stick_x*maxPower), yaw, true);
                telemetry.addLine("Field Centric");
            } else {
                drive.driveRobotCentric((gamepad1.left_stick_y*maxPower), -(gamepad1.left_stick_x*maxPower), (gamepad1.right_stick_x*maxPower), true);
                telemetry.addLine("Robot Centric");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
