package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class outtakeLogic{
     private Motor storage;
     private Motor intake;
     private Motor outtake;
     private ElapsedTime stateTimer = new ElapsedTime();

     private enum OuttakeState {
         IDLE,
         SPIN_UP,
         LAUNCH,
         RESET
     }
     private OuttakeState outtakeState;
     private double SPINUP_TIME = 5;
     private double SHOOT_TIME = 3.5;
     private int shotsRemaining = 0;

     public void init(HardwareMap hardwareMap) {
         storage = new Motor(hardwareMap,"storage");
         intake = new Motor(hardwareMap,"intake");
         outtake = new Motor(hardwareMap, "outtake");

         outtakeState = OuttakeState.IDLE;
         storage.set(0);
         intake.set(0);
         outtake.set(0);
     }

     public void update() {
         switch (outtakeState) {
             case IDLE:
                 if (shotsRemaining > 0) {
                     intake.set(1);
                     outtake.set(1);

                     stateTimer.reset();
                     outtakeState = OuttakeState.SPIN_UP;
                 }
                 break;
             case SPIN_UP:
                 if (stateTimer.seconds() > SPINUP_TIME) {
                     storage.set(1);
                     stateTimer.reset();

                     outtakeState = OuttakeState.LAUNCH;
                 }
                 break;
             case LAUNCH:
                 if (stateTimer.seconds() > SHOOT_TIME) {
                     shotsRemaining = 0;

                     outtakeState = OuttakeState.RESET;
                 }
                 break;
             case RESET:
                 storage.set(0);
                 intake.set(0);
                 outtake.set(0);

                 outtakeState = OuttakeState.IDLE;
                 break;
         }
     }

     public void fire(int numberOfShots) {
         if (outtakeState == OuttakeState.IDLE) {
             shotsRemaining = numberOfShots;
         }
     }

     public boolean isBusy() {
         return outtakeState != OuttakeState.IDLE;
     }
}
