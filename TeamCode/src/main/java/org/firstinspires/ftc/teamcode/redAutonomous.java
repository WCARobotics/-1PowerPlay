package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="redAutonomous")
public class redAutonomous extends LinearOpMode {
   /*public DistanceSensor frontDistance, leftDistance, backDistance, rightDistance;*/
   int redValue, blueValue, greenValue;
   //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
   //expansion hub: arm = 0, arm2 is 1
   public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;
   public CRServo claw, claw2;

   public DistanceSensor backDistanceSensor, leftDistanceSensor, rightDistanceSensor;
   //the variables that keep track of movement
   double armSpeed, turnSpeed;
   //armExtension Value

   //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
   double[] wheels = new double[4];






   public void runOpMode() {

      //initializes the variables
      /*frontDistance = hardwareMap.get(DistanceSensor.class, "distance front");
      leftDistance = hardwareMap.get(DistanceSensor.class, "distance left");
      backDistance = hardwareMap.get(DistanceSensor.class, "distance back");
      rightDistance = hardwareMap.get(DistanceSensor.class, "distance right");
       */
      frontLeftWheel = hardwareMap.get(DcMotor.class, "front left");
      frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
      backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
      backRightWheel = hardwareMap.get(DcMotor.class, "back right");
      arm = hardwareMap.get(DcMotor.class, "arm motor");
      arm2 = hardwareMap.get(DcMotor.class, "arm motor 2");
      claw = hardwareMap.get(CRServo.class, "claw");
      claw2 = hardwareMap.get(CRServo.class, "claw 2");

      backDistanceSensor = hardwareMap.get(DistanceSensor.class, "back distance sensor");
      leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance sensor");
      rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance sensor");

      //set direction for motors
      //Old code: left is forward and right is backward
      //New code: right is forward and left is backwards
      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      backRightWheel.setDirection(DcMotor.Direction.REVERSE);
      arm.setDirection(DcMotorSimple.Direction.REVERSE);
      arm2.setDirection(DcMotorSimple.Direction.REVERSE);

     /*
      double left = leftDistance.getDistance(DistanceUnit.INCH);
      double back = backDistance.getDistance(DistanceUnit.INCH);
      double right = rightDistance.getDistance(DistanceUnit.INCH);*/

      waitForStart();
      boolean clawClosed = true;
      close(500);
      sleep(1000);
      arm("UP", 500);
      sleep(1000);
      //forward
      move(0.5, 0.5, 0.5, 0.5, 50);
      rest(1000);
      //strafe right
      move(0.5, -0.5, -0.5, 0.5, 1000);
      rest(1000);
      //forward
      //arm up
      //approach
      rest(1000);
      //release cone
      open(500);
      sleep(1000);
      //strafe left
      move(-0.5, 0.5, 0.5, -0.5, 1000);
      rest(1000);
      //forward
      move(0.5, 0.5, 0.5, 0.5, 1500);
      //
      //ACTUAL OLD CODE BEFORE MEET 2
      /*
      sleep(50);
      arm("UP", 600);
      //moves forward
      move(0.5, 0.5, 0.5, 0.5, 550);
      //turns 180
    /*  move(-.5, -.5, 0.5, 0.5, 2400);
      rest(1000);


      //moves forward until it detects the cup
      double backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
      //while it stops at 4 inches, the distance is actually a lot shorter because the motors need time to stop moving
      while (backDistance > 5) {
         continuousMove(-0.5, -0.5, -0.5, -0.5);
         backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
      }

      //scan and store the tensor flow detection value in a variable
      //insert code here
      while (backDistance < 5) {
         continuousMove(0.5, 0.5, 0.5, 0.5);
         backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
      }
      */

      //WIP MOVEMENT
      //push the cup
/*
      move(-0.5, -0.5, -0.5, -0.5, 700);
      //move forwards, that last line was meant to push the cup out of the way so it doesn't get ran over and mess up the rest of the lines
      //move(0.5, 0.5, 0.5, 0.5, 500);
      //do a turn 45 degree turn right
      move(0.5, 0.5, -.5, -.5, 800);
      //move back just a bit so it can aline to the lower junction
      move(-.25, -.25, -.25, -.25, 50);
      //waits just a bit bc without it is kinda hectic :)
      sleep(600);
      //moves the arm down so the cone can CLEANLY fall
      arm("DOWN",50);
      //opens the claw
      open(500);
      arm("UP", 100);


      rest(1000);
      move(-0.5, 0.5, .5, .5, 1600);
      move(-0.5, -0.5, -0.5, -0.5, 2000);

      //this long rest is to stop it from running old code
      /*rest(10000000);
      //OLD CODE, MIGHT NOT BE USED BUT KEEPING IT FOR NOW
      //strafe left
      move(-1, 1, 1, -1, 500);
      rest(500);
      //move forward
      move(1, 1, 1, 1, 500);
      //arm up
      //claw release
      up(2000);
      open(1000);
      //move back
      move(-1, -1, -1, -1, 500);
      //arm down
      arm("DOWN",200);
      //strafe right
      move(1, -1, -1, 1, 500);
      rest(500);
      */

   }

   public void move(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, int duration) {
      frontLeftWheel.setPower(frontLeftPower);
      backLeftWheel.setPower(backLeftPower);
      frontRightWheel.setPower(frontRightPower);
      backRightWheel.setPower(backRightPower);
      sleep(duration);
   }
   public void continuousMove(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
      frontLeftWheel.setPower(frontLeftPower);
      backLeftWheel.setPower(backLeftPower);
      frontRightWheel.setPower(frontRightPower);
      backRightWheel.setPower(backRightPower);
   }
   public void arm(String direction, int duration) {
      if (direction.equals("UP")) {
         arm.setPower(0.75);
         arm2.setPower(-0.75);
         sleep(duration);
      }
      else if (direction.equals("DOWN")) {
         arm.setPower(-.5);
         arm2.setPower(0.5);
         sleep(duration);
      }
      arm.setPower(0);
      arm2.setPower(0);
   }
   public void up(int duration){

   }
   public void down(int duration){
      arm.setPower(-1.0);
      arm2.setPower(1.0);
      sleep(duration);
   }
   public void open(int duration){
      claw.setPower(-1);
      claw2.setPower(1);
      sleep(duration);
   }
   public void close(int duration){
      claw.setPower(1);
      claw2.setPower(-1);
      sleep(duration);
   }
   public void rest(int duration) {
      frontLeftWheel.setPower(0);
      backLeftWheel.setPower(0);
      frontRightWheel.setPower(0);
      backRightWheel.setPower(0);
      sleep(duration);
   }

}






