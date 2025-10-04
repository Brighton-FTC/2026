package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurretTester extends OpMode {

    private TurretComponent turret;
    private GamepadEx gamepad;

    private final Pose startingPose = new Pose(0, 0, Math.toRadians(0));



    @Override
    public void init(){
        gamepad = new GamepadEx(gamepad1);
        turret = new TurretComponent(hardwareMap, "turretMotor", 0.05, -72, 72, startingPose);
    }

    @Override
    public void loop(){
        gamepad.readButtons();

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            turret.aimToObject();
        }
    }
}
