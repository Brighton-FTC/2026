package org.firstinspires.ftc.teamcode.FlyWheel;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Configurable
public class FlyWheelMotorPIDComponent {

    private final Motor motor;

    public double kP = 0.0015;
    public double kI = 0 ;
    public double kD = 0;
    public double kS;
    public double kV;
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV);

    private PIDController controller = new PIDController(kP, kI, kD);



    public FlyWheelMotorPIDComponent(HardwareMap hardwareMap, String motorID){
        motor = new Motor(hardwareMap, motorID);
    }

    public void runMotorAt(double velocity){
        controller.setSetPoint(velocity);
        double PIDpower = controller.calculate(motor.getCorrectedVelocity(), velocity);
        double FFpower = ff.calculate(velocity);


        motor.set(FFpower + PIDpower);

    }

    public double getVel(){
        return motor.getCorrectedVelocity();
    }

    public void stopMotor(){
        motor.stopMotor();
    }
}
