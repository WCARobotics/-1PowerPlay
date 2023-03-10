package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele-Op")
public class teleOp extends OpMode {
   //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
   //expansion hub: arm = 0, arm2 is 1
   public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm2;
   //positive claw means open and negative means closed
   public CRServo claw;

   public Servo flipper, leftSide, rightSide;
   //the variables that keep track of movement
   double x, y, theta, power, sin, cos, max, armSpeed, turnSpeed;
   //armExtension Value

   //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
   double[] wheels = new double[4];
   public int leftFrontPos;
   public int rightFrontPos;
   public int leftBackPos;
   public int rightBackPos;
   public int armLeftPos;
   public int armRightPos;
   public int total;
   public ElapsedTime mStateTime = new ElapsedTime();
   public int servoState = 2;
   public int servoOpen = 1;
   public void init() {
      //initializes the variables
      frontLeftWheel = hardwareMap.get(DcMotor.class,  "front left");
      frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
      backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
      backRightWheel = hardwareMap.get(DcMotor.class, "back right");

      arm2 = hardwareMap.get(DcMotor.class, "left arm motor");
      claw = hardwareMap.get(CRServo.class, "claw");
      flipper = hardwareMap.get(Servo.class, "flipper");
      leftSide = hardwareMap.get(Servo.class, "leftSide");
      rightSide = hardwareMap.get(Servo.class, "rightSide");
      arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      backRightWheel.setDirection(DcMotor.Direction.REVERSE);

      arm2.setDirection(DcMotorSimple.Direction.REVERSE );
      flipper.setDirection(Servo.Direction.FORWARD);
      leftSide.setDirection(Servo.Direction.FORWARD);
      rightSide.setDirection(Servo.Direction.REVERSE);



      leftFrontPos = 0;
      rightFrontPos = 0;
      leftBackPos = 0;
      rightBackPos = 0;
      armLeftPos = 0;
      total = 0;




   }

   public void loop() {

      //coordinate position of the x value of the joystick
      x = gamepad1.left_stick_x;
      //coordinate position of the y value of the joystick
      y = -gamepad1.left_stick_y;
      //This is to tell the angle of the joystick away from the positive x-coordinate line ranging from 0to 360 degrees to then be turned into actual coordinates
      theta = Math.atan2(y, x);
      //To tell how far away the joystick is from the center to tell how much power the driver wants in a certain direction
      power = Math.hypot(x, y);
      //Trig functions for joystick movement math
      sin = Math.sin(theta - Math.PI / 4);
      cos = Math.cos(theta - Math.PI / 4);
      max = Math.max(Math.abs(sin), Math.abs(cos));
      //speed of the arms
      armSpeed = 0.5;
      //This is meant to always reset the turn speed so if the button is not pressed then it will not keep going
      turnSpeed = 0.0;


      //controls when the robot turns by setting turnSpeed to a different value
      if (gamepad1.left_bumper) {
         turnSpeed = -1;
      } else if (gamepad1.right_bumper) {
         turnSpeed = 1 ;
      }


      for (int i = 0; i < wheels.length; i++) {
         //code for the front right and back left wheels
         if (i % 2 == 0) {
            wheels[i] = (power * (sin / max));
            //This is meant to tell whether the wheel is right or left to then tell how the turn speed will affect it because right
            // have to be subtracted and left is added and the negative turn speed will deal with if the other bumper is pressed
            if (i == 0) {
               wheels[0] -= turnSpeed;
            } else {
               wheels[2] += turnSpeed;
            }
         }
         //code for the front left and back right wheels
         else {
            wheels[i] = (power * (cos / max));
            if (i == 3) {
               wheels[3] -= turnSpeed;
            } else {
               wheels[1] += turnSpeed;
            }
         }
         //This is meant to keep the ratios the same so that if it does go over the 1 then the ratio will stay
         // the same and kept as full power as they can be
         if ((power + Math.abs(turnSpeed)) > 1) {
            wheels[i] /= power + turnSpeed;
         }
      }
      move(wheels);
      //if both triggers are not pressed at the same time
      if (!(gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0)) {
         //turn up

         if (gamepad1.right_trigger > 0) {
            //UP

            armLeftPos += 11.5;
            if(armLeftPos >= 11500){
               armLeftPos = 11500;
            }

            arm2.setTargetPosition(armLeftPos);

            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm2.setPower(1);
            //armExtensionValue +=0.1;
         } else if (gamepad1.left_trigger > 0) {
            //DOWN
            armLeftPos -= 11.5;
            if(armLeftPos <= 0){
               armLeftPos = 0;
            }

            arm2.setTargetPosition(armLeftPos);

            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm2.setPower(1);


         } else {



         }
      } else {



      }
      //claw controls
      if(gamepad1.b){
         claw.setPower(-.5);
         mStateTime.reset();
         servoOpen = 0;
      }else if(gamepad1.x){
         claw.setPower(.5);

      }
      if(servoOpen == 0 && mStateTime.time() >= 1){
         claw.setPower(0);
         servoOpen = 1;
      }
      //flipping controls
      if(gamepad1.y){
         flipper.setPosition(0);
         rightSide.setPosition(1);
         leftSide.setPosition(1);
         mStateTime.reset();
         servoState = 1;
      }else if (gamepad1.a){
         flipper.setPosition(1);


         mStateTime.reset();
         servoState = 0;
      }
      if(servoState == 1 && mStateTime.time() >= 1 ){
         rightSide.setPosition(1);
         leftSide.setPosition(1);
         servoState  = 2;
      }else if(servoState == 0 && mStateTime.time() >= 1){
         rightSide.setPosition(0);
         leftSide.setPosition(0);
         servoState  = 2;
      }



   }
   //method for setting each wheel's power in order for the robot to move properly
   public void move ( double[] wheels){
      frontLeftWheel.setPower(wheels[1]);
      backLeftWheel.setPower(wheels[2]);
      frontRightWheel.setPower(wheels[0]);
      backRightWheel.setPower(wheels[3]);
   }

}


