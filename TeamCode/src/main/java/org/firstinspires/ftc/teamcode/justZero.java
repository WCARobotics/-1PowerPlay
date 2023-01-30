
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "justZero")
public class justZero extends LinearOpMode {
   //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
   //expansion hub: arm 2 = 1, claw = 0, flipper = 1, leftSide = 2,
   public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm2;
   public CRServo claw;
   public Servo flipper, leftSide, rightSide;


   //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
   double[] wheels = new double[4];
   public int leftFrontPos;
   public int rightFrontPos;
   public int leftBackPos;
   public int rightBackPos;
   public int armLeftPos;
   private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/powerPlayTensorflow.tflite";


   private static final String[] LABELS = {
           "Number One",
           "Number Three",
           "Number Two"
   };

   private static final String VUFORIA_KEY =
           "AY+LAO//////AAABmXqpfGFoHkhWrS1QLc/3RBMitc3sWaySydW66iqgCgULFQBcw/vHIj1o9YeqLzsHNYgoJ4bigISHSEJ0aqFhiZ1r+rRJ0HhFgL4V88oT/6FlJFzRDwhtVX+72HEoEIZgiH2vZD5i5mQp11U9rz+oE/07CwUzmABaW9i4gI50lPJhve1K6xRd4ydgjVhdiJ/Ayz6X8mK+LBnX10KxS0cCEtbCi2texH/X9W+iMXoXk9bzfnKi8X0xJxOAO9F5R2Ja9wBvEZrQQFhf/e4wegFY3fTYqsKtRTaENLYtQFuPqelbRuFRR2qcIZ9Q677IXEM+Ydagzu1bj2lBc6ueZf5SYn0iaPOIWRp4ilLNlRj5wyZ4";

   private VuforiaLocalizer vuforia;

   private TFObjectDetector tfod;

   public void runOpMode() {
      frontLeftWheel = hardwareMap.get(DcMotor.class, "front left");
      frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
      backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
      backRightWheel = hardwareMap.get(DcMotor.class, "back right");
      arm2 = hardwareMap.get(DcMotor.class, "left arm motor");
      claw = hardwareMap.get(CRServo.class, "claw");
      flipper = hardwareMap.get(Servo.class, "flipper");
      leftSide = hardwareMap.get(Servo.class, "leftSide");
      rightSide = hardwareMap.get(Servo.class, "rightSide");

      frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
      backRightWheel.setDirection(DcMotor.Direction.REVERSE);
      arm2.setDirection(DcMotorSimple.Direction.REVERSE);
      flipper.setDirection(Servo.Direction.FORWARD);
      leftSide.setDirection(Servo.Direction.FORWARD);
      rightSide.setDirection(Servo.Direction.REVERSE);

      leftFrontPos = 0;
      rightFrontPos = 0;
      leftBackPos = 0;
      rightBackPos = 0;
      armLeftPos = 0;

      waitForStart();
      flipper.setPosition(0);
   }
}