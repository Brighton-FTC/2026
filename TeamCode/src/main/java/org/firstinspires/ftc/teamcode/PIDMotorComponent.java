package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PIDMotorComponent {

    private Motor motor;
    private MultipleTelemetry telemetry;

    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public PIDMotorComponent(HardwareMap hardwareMap, String motorID, Telemetry telemetry){
        motor = new Motor(hardwareMap, motorID);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        motor.resetEncoder();
    }

    public void startVelControlTest(){
        telemetry.update();
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(kP, kI, kD);
        motor.setFeedforwardCoefficients(kS, kV, kA);

        motor.set(1);
        telemetry.addData("Velocity", motor.getCorrectedVelocity());
    }


    public void startPosControlTest(){
        telemetry.update();
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionCoefficient(kP);

        motor.setDistancePerPulse(20);
        motor.setTargetPosition(1200);
        motor.set(1);
        telemetry.addData("Position", motor.getCurrentPosition());
    }

}
