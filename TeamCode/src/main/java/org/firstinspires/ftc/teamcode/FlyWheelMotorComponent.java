package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheelMotorComponent {

    private final Motor motor;

    public FlyWheelMotorComponent(HardwareMap hardwareMap, String motorID){
        motor = new Motor(hardwareMap, motorID);
        motor.setRunMode(Motor.RunMode.VelocityControl);
    }

    public void runMotorAt(double velocity){
        motor.set(velocity);
    }
}
