// Autonomous program that leverages a Mecanum drive train.
// For De La Salle - Method parameters specify each motor's behavior.
//
// Ed C. Epp
// January, 11 2023
// March, 19 2023    After software upgrade and testing tweaks
// March, 24, 2024   Simplified powerUpMotors to use Ticks
//                   Add a couple of helper methods: 
//                       GoForward and GoSideways
// Sophia Rangel Carmona- program parks robot in left terminal then resets itself
//                      Methods Added: RightRotate/LeftRotate

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Terminal Park", group="Concept")
//@Disabled
// --------------------------- DriveInASquareMecanumSR class -----------------------------
// ------------------------------------------------------------------------------
public  class TerminalPark extends LinearOpMode
{
    int TARGET_DISTANCE      = 225;
    int TARGET_DISTANCE2     = 2940;
    int TARGET_TURN          = 1170;
    // This is a quess because the wheels may slip
    double DEGREES_90         = 125;         // wheel distance guess
    double TARGET_VELOCITY    = 1000;
    int    SQUARE_SIDES       =   1;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Configure the motors
        leftMotor = hardwareMap.get(DcMotorEx.class,"myLeftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class,"myRightMotor");
        leftMotor2 = hardwareMap.get(DcMotorEx.class,"myLeftMotor2");
        rightMotor2 = hardwareMap.get(DcMotorEx.class,"myRightMotor2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        // Move
       GoForward (TARGET_DISTANCE, TARGET_VELOCITY);
       LeftRotate (TARGET_TURN, TARGET_VELOCITY);
       GoForward (TARGET_DISTANCE2, TARGET_VELOCITY);
       GoForward (-TARGET_DISTANCE2, TARGET_VELOCITY);
       RightRotate (TARGET_TURN, TARGET_VELOCITY);
       GoForward (-TARGET_DISTANCE, TARGET_VELOCITY);
    }
    
    // ---------- RightRotate -------------------------------------
    void RightRotate (int Distance, double Velocity)
    {
        powerUpMotors (Distance, Velocity,
                        -Distance, -Velocity,
                        Distance, Velocity,
                        -Distance, Velocity);
    }
    
    // ---------- LeftRotate ---------------------------------------
    void LeftRotate (int Distance, double Velocity)
    {
        powerUpMotors(-Distance, -Velocity,
                        Distance, Velocity,
                        -Distance, -Velocity,
                        Distance, Velocity);
    }
    
    // ---------- GoForward -------------------------------------
    void GoForward (int Distance, double Velocity)
    {
             powerUpMotors(Distance, Velocity,
                             Distance, Velocity,
                             Distance, Velocity,
                             Distance, Velocity);
    }

    // ---------- GoSideways -------------------------------------
    void GoSideways (int Distance, double Velocity)
    {
             powerUpMotors(-Distance, -Velocity,
                           Distance, Velocity,
                           Distance, Velocity,
                           -Distance, -Velocity);
    }

    // ---------- powerUpMotors -----------------------------------
    // Turn each moter a set distance and a velocity it should acheive
    //    leftMotorDistance:   encoder ticks
    //    velocityMm:          target velocity for that wheel in ticks  
    //                              per second
 
    void powerUpMotors (int leftMotorDistance, double leftMotorVelocity,
                        int rightMotorDistance, double rightMotorVelocity, 
                        int leftMotor2Distance, double leftMotor2Velocity, 
                        int rightMotor2Distance, double rightMotor2Velocity)
    {
        // reset the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
       // set the possition to which each wheel should run
        leftMotor.setTargetPosition(leftMotorDistance);
        rightMotor.setTargetPosition(rightMotorDistance);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);    
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setTargetPosition(leftMotor2Distance);
        rightMotor2.setTargetPosition(rightMotor2Distance);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);    
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the velocity of each wheel
        leftMotor.setVelocity((int)leftMotorVelocity);
        rightMotor.setVelocity(rightMotorVelocity);
        leftMotor2.setVelocity(leftMotor2Velocity);
        rightMotor2.setVelocity(rightMotor2Velocity);

        // do nothing until each wheel has completed it mission
        while (leftMotor.isBusy() || rightMotor.isBusy() ||
               leftMotor2.isBusy() ||  rightMotor2.isBusy ())
        {
            telemetry.addData("position  velocity ", 
               leftMotor.getCurrentPosition() + "   " + leftMotor.getVelocity() + "  " +
               rightMotor.getCurrentPosition() + "  " + rightMotor.getVelocity() + "  " +
               leftMotor2.getCurrentPosition() + "   " + leftMotor2.getVelocity() + "  " +
               rightMotor2.getCurrentPosition() + "  " + rightMotor2.getVelocity());
            telemetry.update();
            idle();
        }
        // Motors are off - not busy
    }
}
