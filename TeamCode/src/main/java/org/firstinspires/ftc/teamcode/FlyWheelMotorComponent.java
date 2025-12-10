package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Configurable
public class FlyWheelMotorComponent {

    private final Motor motor;

    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;



    public FlyWheelMotorComponent(HardwareMap hardwareMap, String motorID){
        motor = new Motor(hardwareMap, motorID);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(kP, kI, kD);
        motor.setFeedforwardCoefficients(kS, kV);

    }

    public void runMotorAt(double velocity){
        motor.set(velocity);

    }

    public void stopMotor(){
        motor.stopMotor();
    }
}
