package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class RedTeleop extends GenericTeleop {
    protected double getObjectXPosition() {
        return 144;
    }

    protected Pose getStartingPose() {
        return new Pose(132, 132, Math.toRadians(225));
    }
}