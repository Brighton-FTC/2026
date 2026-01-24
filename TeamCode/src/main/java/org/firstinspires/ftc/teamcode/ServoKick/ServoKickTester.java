package org.firstinspires.ftc.teamcode.ServoKick;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

@TeleOp
public class ServoKickTester extends OpMode{

    //to use init and loop we have to extend OpMode.

    //init only get to run once at the start which sets everything up
    //loop runs in cycle just like a while loop.

    //forgot this again lmao
    private ServoKickComponent servo1;

    private GamepadEx gamepad;

    private boolean up;

    @Override
    public void init() {
        //actually we just need one in this case because we are testing if we can activate servo or not
        //In the real production code we need 3 objects just like this.
        servo1 = new ServoKickComponent(hardwareMap, "servo1");
        gamepad = new GamepadEx(gamepad1);

        servo1.down();
        up = false;
    }

    @Override
    public void loop() {
        gamepad.readButtons(); //Must call this at the start of the loop otherwise buttons won't be read.
        if (gamepad.wasJustPressed(PSButtons.CIRCLE) && up){
            servo1.down();
            up = false;
        }
        //to be more efficient, we can use boolean.
        if (gamepad.wasJustPressed(PSButtons.SQUARE)&& !up){
            servo1.up();
            up = true;
        }
        if (gamepad.wasJustPressed(PSButtons.CIRCLE)){
            telemetry.addData("pos", servo1.getServoStatus());
            //here we can output it for debugging with telemetry but i need to read docs for this because its pain in the ass...
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            servo1.up();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servo1.down();
        }

    }


    //We have 3 servos for 3 artifacts this game.

}
