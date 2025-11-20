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

public class IndexerComponent {
    public boolean[] targetColours = {true, true, false};
    public boolean[] colours = {false, false, false};
    public boolean[] used = {false, false, false};

    public int currentBall = 0;

    public Servo[] servos = {null, null, null};
    public ColorSensor[] sensors = {null, null, null};

    public static final int servoAngle = 100;

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
                used[i] = false;
                if (rb > g) {
                    // Purple
                    colours[i] = true;
                }
                {
                    // Green
                    colours[i] = false;
                }
            } else {
                used[i] = true;
            }
        }
    }

    public void fire() {
        sense();


        currentBall++;
    }

}