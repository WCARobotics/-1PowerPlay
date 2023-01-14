package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="strafeLeft")
public class strafeLeft extends LinearOpMode {


   //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
   //expansion hub: arm = 0, arm2 is 1
   public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;
   public CRServo claw;

   //the variables that keep track of movement
   double armSpeed, turnSpeed;
   //armExtension Value

   //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
   double[] wheels = new double[4];

   public void runOpMode() {
      //initializes the variables

      frontLeftWheel = hardwareMap.get(DcMotor.class, "front left");
      frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
      backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
      backRightWheel = hardwareMap.get(DcMotor.class, "back right");
      arm = hardwareMap.get(DcMotor.class, "arm motor");
      arm2 = hardwareMap.get(DcMotor.class, "arm motor 2");
      claw = hardwareMap.get(CRServo.class, "claw");

      //set direction for motors
      //Old code: left is forward and right is backward
      //New code: right is forward and left is backwards
      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      backRightWheel.setDirection(DcMotor.Direction.REVERSE);
      arm.setDirection(DcMotorSimple.Direction.FORWARD);
      arm2.setDirection(DcMotorSimple.Direction.FORWARD);

      close();
      move(-.75, .75, .75,-.75, 1000);
      rest(1000);
   }

   public void move(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, int duration) {
      frontLeftWheel.setPower(frontLeftPower);
      backLeftWheel.setPower(backLeftPower);
      frontRightWheel.setPower(frontRightPower);
      backRightWheel.setPower(backRightPower);
      sleep(duration);
   }
   public void up(int duration){
      arm.setPower(0.7);
      arm2.setPower(-0.5);
      sleep(duration);
   }
   public void down(int duration){
      arm.setPower(-0.7);
      arm2.setPower(0.5);
      sleep(duration);
   }
   public void open(){
      claw.setPower(-1);

   }
   public void close(){
      claw.setPower(1);

   }
   public void rest(int duration) {
      frontLeftWheel.setPower(0);
      backLeftWheel.setPower(0);
      frontRightWheel.setPower(0);
      backRightWheel.setPower(0);
      sleep(duration);
   }
}

