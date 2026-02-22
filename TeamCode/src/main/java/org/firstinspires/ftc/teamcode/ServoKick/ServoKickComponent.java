package org.firstinspires.ftc.teamcode.ServoKick;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoKickComponent {
    private final Servo kicker; //forgot to set this first, telling java we have this item...
    //Constructor, public + class name. arguments in () i forgo how servos work lmao, aparently it wworks same as motor...
    public ServoKickComponent(HardwareMap hardwareMap, String servoID){
        kicker = hardwareMap.servo.get(servoID);

    }

    public void up(){
        kicker.setPosition(kicker.getPosition()+0.15);
    }
    //Either 0 or 1. 1 means full rotation (servo we have only ranges [0, 180] degrees. Its good to work in radians too [0, pi])
    public void down(){
        kicker.setPosition(kicker.getPosition()-0.15);
    }


    //when you want a function to output a number we replace void with a data type. void means output nothing.
    public double getServoStatus(){
        return kicker.getPosition();
    }
}
