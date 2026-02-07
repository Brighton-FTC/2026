package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="example auto using autoopmode ")
public class ExampleBlueAuto extends AutoOpmode {
    public ExampleBlueAuto(){
        super(
                new Pose(21.751351351351364,124.95135135135135,Math.toRadians(142)),
                new Pose(50.20540540540541,93.98918918918922,Math.toRadians(133)),
                new Pair[]{
                        new Pair<>(
                                new Pose(44.08275862068965, 83.91724137931034, Math.toRadians(180)),
                                new Pose(15.393103448275863, 83.91724137931034, Math.toRadians(180))
                        ),
                        new Pair<>(
                                new Pose(44.08275862068965, 60, Math.toRadians(180)),
                                new Pose(15.393103448275863, 60, Math.toRadians(180))
                        ),
                        new Pair<>(
                                new Pose(44.08275862068965, 35, Math.toRadians(180)),
                                new Pose(15.393103448275863, 35, Math.toRadians(180))
                        )
                }

        );

    }
    @Override
    public void init(){
        super.init();
        //add intake and shooter initialization here if not added in the AutoOpmode class
        telemetry.addLine("Blue auto 1 initialsed");
    }

    @Override
    public void loop(){
        super.loop();
    }
}
