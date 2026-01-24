package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class DrivetrainTest extends OpMode {
    private MecanumDrive drive;
    private GamepadEx gamepadEx1;

    private Motor lf;
    private Motor rf;
    private Motor rr;
    private Motor lr;

    @Override
    public void init() {
        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        rr = new Motor(hardwareMap, "rr");
        lr = new Motor(hardwareMap, "lr");
        lf.setInverted(true);
        drive = new MecanumDrive(lf, rf, lr, rr);
        gamepadEx1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX()
        );

        telemetry.addData("leftfront", lf.getCurrentPosition());
        telemetry.addData("rightfront", rf.getCurrentPosition());
        telemetry.addData("leftback", lr.getCurrentPosition());
        telemetry.addData("rightback", rr.getCurrentPosition());
        telemetry.update();

    }
}
