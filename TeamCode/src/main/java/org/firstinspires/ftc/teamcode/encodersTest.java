
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "encodersTest")
public class encodersTest extends LinearOpMode {


    //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
    //expansion hub: arm = 0, arm2 is 1
    public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;


    public DistanceSensor backDistanceSensor, leftDistanceSensor, rightDistanceSensor;
    //the variables that keep track of movement
    double armSpeed, turnSpeed;
    //armExtension Value

    //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
    double[] wheels = new double[4];
    public int leftPos;
    public int rightPos;
    public int armPos;

    public void runOpMode() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "front left"); // 1 on the expansion hub for config
        frontRightWheel = hardwareMap.get(DcMotor.class, "front right"); // 0 on the expansion hub for config
        backLeftWheel = hardwareMap.get(DcMotor.class, "back left"); // 2 on the expansion hub for config
        backRightWheel = hardwareMap.get(DcMotor.class, "back right"); // 3 on the expansion hub for config
        arm = hardwareMap.get(DcMotor.class, "right arm motor"); //0 on the control hun for config
        arm2 = hardwareMap.get(DcMotor.class, "left arm motor"); //1 on the control hub for config
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*backDistanceSensor = hardwareMap.get(DistanceSensor.class, "back distance sensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance sensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance sensor");

         */

        //set direction for motors
        //Old code: left is forward and right is backward
        //New code: right is forward and left is backwards
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPos = 0;
        rightPos = 0;
        armPos = 0;
        waitForStart();
        //Get into position to aim the camera
        arm("UP", 550);
        forward(1,100);
        rest(200);
        flip();
        rest(500);
        //Move the robot to the high junctions and then drop a cone
        flip();
        rest(500);


        forward(1,6000);
        rest(500);
        arm("UP", 250);
        turnLeft(1,300);
        forward(1,400);

        rest(500);
        //Center the robot on the mat to then go and pick up a new cone
        backward(1,500);
        turnRight(1,400);
        rightNine();
        forward(1,1000);
        //Move the robot back to the high junction
        backward(1,1100);
        leftNine();
        turnLeft(1,600);
        forward(1,400);
        //Move to the targetted parking space








    }
    //functions
    public void move(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, int target) {
        leftPos += target;
        rightPos += target;
        //Setting the target positions
        frontLeftWheel.setTargetPosition(leftPos);
        frontRightWheel.setTargetPosition(rightPos);
        backLeftWheel.setTargetPosition(leftPos);
        backRightWheel.setTargetPosition(rightPos);
        //Setting the mode
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Setting the power of the motor
        frontLeftWheel.setPower(frontLeftPower);
        backLeftWheel.setPower(backLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backRightWheel.setPower(backRightPower);
        while(opModeIsActive() && motorActive()){
            idle();
        }
    }
    public boolean motorActive(){
        if(frontLeftWheel.isBusy() && frontRightWheel.isBusy()){
            if(backLeftWheel.isBusy() && backRightWheel.isBusy()){
                return true;
            }else{
                return false;
            }
        }else{
            return false;
        }
    }

    public void forward(double power, int duration){
        move(power,power,power,power,duration);
    }
    public void backward(double power, int duration){
        move(-power,-power,-power,-power,duration);
    }
    public void right(double power, int duration){
        move(power,-power,-power,power,duration);
    }
    public void left(double power, int duration){
        move(-power,power,power,-power,duration);
    }
    public void turnRight(double power, int duration){
        move(power,power,-power,-power,duration);
    }
    public void turnLeft(double power, int duration){
        move(-power,-power,power,power,duration);
    }
    public void flip(){
        move(-1,-1,1,1, 1750);
    }
    public void rightNine(){
        move(1,1,-1,-1, 1000);
    }
    public void leftNine(){
        move(-1,-1,1,1, 750);
    }
    public void continuousMove(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftWheel.setPower(frontLeftPower);
        backLeftWheel.setPower(backLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backRightWheel.setPower(backRightPower);
    }
    public void arm(String direction, int target) {
        if (direction.equals("UP")) {
            armPos += target;
            arm.setTargetPosition(armPos);
            arm2.setTargetPosition(-armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(.1);
            arm2.setPower(0.1);
            while(opModeIsActive() && armActive()){
                idle();
            }
        }
        else if (direction.equals("DOWN")) {
            armPos += target;
            arm.setTargetPosition(-armPos);
            arm2.setTargetPosition(armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(.1);
            arm2.setPower(0.1);
            while(opModeIsActive() && armActive()){
                idle();
            }
        }

    }
    public boolean armActive(){
        if(arm.isBusy() && arm2.isBusy()){
            return true;
        }else{
            return false;
        }
    }



    public void rest(int duration) {
        frontLeftWheel.setPower(0);
        backLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backRightWheel.setPower(0);
        sleep(duration);
    }
}
