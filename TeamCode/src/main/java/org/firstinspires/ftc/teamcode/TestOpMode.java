package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestOpMode", group="Iterative OpMode")
@Disabled
public class TestOpMode extends OpMode
{
    // Declare OpMode members.


    public static final Ball[] targetColours = {Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
    public int currentBall = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private double time;

    private boolean ready = true;
    private boolean pressed = false;

    private IndexerComponent indexer;
    private Gamepad gamepad;


    @Override
    public void init() {
        indexer = new IndexerComponent();
        gamepad = new Gamepad();
    }

    @Override
    public void loop() {
        if (gamepad.a) {
            if (!pressed && ready) {
                indexer.fire(
                        targetColours[currentBall % targetColours.length]
                );
                currentBall++;
                time = getRuntime();
                ready = false;
            }
            pressed = true;
        } else {
            pressed = false;
        }

        if (getRuntime() > time + 2.0f && !ready) {
            indexer.reset();
            ready = true;
        }
    }
}
