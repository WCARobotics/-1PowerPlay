package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="justArms")
public class justArms extends OpMode {
    //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
    //expansion hub: arm = 0, arm2 is 1
    public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm2;
    //posiitive claw means open and negative means closed
    public CRServo claw;

    public Servo flipper, leftSide, rightSide;
    /*public CRServo leftSide, rightSide, claw;
    public Servo flipper;

     */
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


      /*
      claw = hardwareMap.get(CRServo.class, "claw");
      flipper = hardwareMap.get(Servo.class, "flipper");
      leftSide = hardwareMap.get(CRServo.class, "leftSide");
      rightSide = hardwareMap.get(CRServo.class, "rightSide");

       */




        //set direction for motors
        //Old code: left is forward and right is backward
        //New code: right is forward and left is backwards
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        arm2.setDirection(DcMotorSimple.Direction.REVERSE );
        flipper.setDirection(Servo.Direction.FORWARD);
        leftSide.setDirection(Servo.Direction.FORWARD);
        rightSide.setDirection(Servo.Direction.REVERSE);


      /*
      flipper.setPosition(0);

       */
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;
        armLeftPos = 0;
        total = 0;




    }

    public void loop() {


        //if both triggers are not pressed at the same time
        if (!(gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0)) {
            //turn up
            //if(gamepad1.right_trigger > 0 && armExtensionValue<2){
            if (gamepad1.right_trigger > 0) {
                //UP



                arm2.setPower(1);
                //armExtensionValue +=0.1;
            } else if (gamepad1.left_trigger > 0) {


                arm2.setPower(-1);


            } else {
                arm2.setPower(0);


            }
        } else {

            arm2.setPower(0);

        }

    }
    //method for setting each wheel's power in order for the robot to move properly
    public void move ( double[] wheels){
        frontLeftWheel.setPower(wheels[1]);
        backLeftWheel.setPower(wheels[2]);
        frontRightWheel.setPower(wheels[0]);
        backRightWheel.setPower(wheels[3]);
    }
  /* public void flipUp(){
      rightSide.setPower(1);
      leftSide.setPower(-1);

      rightSide.setPower(0);
      leftSide.setPower(0);
   }
   public void flipDown(){
      rightSide.setPower(-1);
      leftSide.setPower(1);

      rightSide.setPower(0);
      leftSide.setPower(0);
   }
   public void open(){
      claw.setPower(.75);


   }
   public void close(){
      claw.setPower(-.75);


   }

   */
}


