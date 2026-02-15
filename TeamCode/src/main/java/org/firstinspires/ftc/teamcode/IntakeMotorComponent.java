package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotorComponent {

    private final Motor motor;
    public IntakeMotorComponent(HardwareMap hardwareMap, String motorID){
        motor = new Motor(hardwareMap, motorID);
        motor.setRunMode(Motor.RunMode.RawPower);

    }

    public void startMotor(){
        motor.set(-1);
    }

    public void stopMotor(){
        motor.stopMotor();
    }

}
