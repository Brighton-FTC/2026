package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class BlueTeleop extends GenericTeleop {
    protected double getObjectXPosition() {
        return 0;
    }

    protected Pose getStartingPose() {
        return new Pose(12, 132, Math.toRadians(315));
    }
}