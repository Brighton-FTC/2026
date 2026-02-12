package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurretTester extends OpMode {

    private TurretPIDComponent turret;
    private GamepadEx gamepad;

    public final Pose startingPose = new Pose(12, 132, Math.toRadians(315));

    public boolean run = false;



    @Override
    public void init(){
        gamepad = new GamepadEx(gamepad1);
        //Scaling factor == 3.214 in degrees; 0.0561 in radians. (360*0.25/28)
        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, 0, 144, telemetry);
    }

    @Override
    public void loop(){
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            run = true;
        }
        if (run){
            turret.aimToObject(12, 132, Math.toRadians(315));
        }
        telemetry.update();
    }
}
