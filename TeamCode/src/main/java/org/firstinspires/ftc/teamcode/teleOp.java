package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="Tele-Op")
public class teleOp extends OpMode {
   //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
   //expansion hub: arm = 0, arm2 is 1
   public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;

   //the variables that keep track of movement
   double x, y, theta, power, sin, cos, max, armSpeed, turnSpeed;
   //armExtension Value

   //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
   double[] wheels = new double[4];

   public void init() {
      //initializes the variables
      frontLeftWheel = hardwareMap.get(DcMotor.class, "front left");
      frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
      backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
      backRightWheel = hardwareMap.get(DcMotor.class, "back right");
      arm = hardwareMap.get(DcMotor.class, "right arm motor");
      arm2 = hardwareMap.get(DcMotor.class, "left arm motor");



      //set direction for motors
      //Old code: left is forward and right is backward
      //New code: right is forward and left is backwards
      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      backRightWheel.setDirection(DcMotor.Direction.REVERSE);
      arm.setDirection(DcMotorSimple.Direction.REVERSE);
      arm2.setDirection(DcMotorSimple.Direction.REVERSE);
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
      //Trig functions for joystick movement math ???????????????????
      sin = Math.sin(theta - Math.PI / 4);
      cos = Math.cos(theta - Math.PI / 4);
      max = Math.max(Math.abs(sin), Math.abs(cos));
      //speed of the arms
      armSpeed = 0.5;
      //This is meant to always reset the turn speed so if the button is not pressed then it will not keep going
      turnSpeed = 0.0;


      //controls when the robot turns by setting turnSpeed to a different value
      if (gamepad1.left_bumper) {
         turnSpeed = -0.75;
      } else if (gamepad1.right_bumper) {
         turnSpeed = 0.75 ;
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
         //if(gamepad1.right_trigger > 0 && armExtensionValue<2){
         if (gamepad1.right_trigger > 0) {
            arm.setPower(armSpeed);
            arm2.setPower(-armSpeed);
            //armExtensionValue +=0.1;
         } else if (gamepad1.left_trigger > 0) {
            arm.setPower(-armSpeed);
            arm2.setPower(armSpeed);
            //armExtensionValue -=0.1;
            //if (armExtensionValue <0) {
            //    armExtensionValue = 0;
            //}
         } else {
            arm.setPower(0);
            arm2.setPower(0);
         }
      } else {
         arm.setPower(0);
         arm2.setPower(0);
      }
      //claw controls
      /*if(gamepad1.x){
         claw.setPower(1.0);
         claw2.setPower(-1.0);
      }else if(gamepad1.b) {
         claw.setPower(-1.0);
         claw2.setPower(1.0);
      }
*/
   }
   //method for setting each wheel's power in order for the robot to move properly
   public void move ( double[] wheels){
      frontLeftWheel.setPower(wheels[1]);
      backLeftWheel.setPower(wheels[2]);
      frontRightWheel.setPower(wheels[0]);
      backRightWheel.setPower(wheels[3]);
   }
}


