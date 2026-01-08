package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

enum Ball {
    GREEN,
    PURPLE,
    EMPTY
}

public class IndexerComponent {
    //////////////////// BALLS ////////////////////
    // Array representing the colours of current balls in storage
    public Ball[] colours = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};


    /////////////////// HARDWARE ///////////////////
    // Array holding the servos
    public Servo[] servos = {null, null, null};
    // Array holding the colour sensors
    public ColorSensor[] sensors = {null, null, null};

    /////////////////// CONSTANTS ///////////////////
    public static final float servoAngle = 0.5f;

    public IndexerComponent() {
        servos[0] = hardwareMap.servo.get("indexerservo1");
        servos[1] = hardwareMap.servo.get("indexerservo2");
        servos[2] = hardwareMap.servo.get("indexerservo3");

        sensors[0] = hardwareMap.colorSensor.get("indexersensor1");
        sensors[1] = hardwareMap.colorSensor.get("indexersensor2");
        sensors[2] = hardwareMap.colorSensor.get("indexersensor3");
    }

    public void update() {
    }

    public void sense() {
        for (int i = 0; i<2; i++) {
            int rb = (sensors[i].red()+sensors[i].blue())/2;
            int g = (sensors[i].green());
            if ((sensors[i].blue()+sensors[i].red()+sensors[i].green())/3>50) {
                if (rb > g) {
                    // Purple
                    colours[i] = Ball.PURPLE;
                }
                {
                    // Green
                    colours[i] = Ball.GREEN;
                }
            } else {
                colours[i] = Ball.EMPTY;
            }
        }
    }

    public void fire(Ball targetColour) {
        // Detect the balls in storage
        sense();

        // Initialise slot
        int slot = -1;

        // Loop over all of the slots and find the ones that have the right colour of ball in it.
        for (int i = 0; i<3; i++) {
            if (colours[i] == targetColour) {
                slot = i;
                break;
            }
        }

        servos[slot].setPosition(servoAngle);
    }

    public void reset() {
        for (int i = 0; i<3; i++) {
            servos[i].setPosition(0);
        }
    }

}