
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "somethingCool")

public class somethingCool extends LinearOpMode {
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
   private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/model_20221218_092054.tflite";


   private static final String[] LABELS = {
           "First",
           "Second",
           "Third"
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



      initVuforia();
      initTfod();
      if (tfod != null) {
         tfod.activate();
         tfod.setZoom(1.0, 16.0/9.0);
      }
      telemetry.addData(">", "Press Play to start op mode");
      telemetry.update();
      waitForStart();
      // put the code that goes before the checking here
      //close the arm to grab the cone and then pick it up and then move forward to the cone and then do a 180 rotation to scan
      close(500);
      sleep(750);
      arm("UP", 250);
      sleep(750);
      //forward
      move(0.5, 0.5, 0.5, 0.5, 750);
      rest(500);
      //does a 180 to scan
      move(1, 1, -1, -1, 1000);
      rest(500);
      String recogniton1 = "";
      int count = 0;
      while(recogniton1.equals("") && count < 4){
            if (tfod != null) {
               List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
               if (updatedRecognitions != null) {
                  telemetry.addData("# Objects Detected", updatedRecognitions.size());

                  for (Recognition recognition : updatedRecognitions) {
                     double col = (recognition.getLeft() + recognition.getRight()) / 2;
                     double row = (recognition.getTop() + recognition.getBottom()) / 2;
                     double width = Math.abs(recognition.getRight() - recognition.getLeft());
                     double height = Math.abs(recognition.getTop() - recognition.getBottom());
                     telemetry.addData("", " ");
                     telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                     telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                     telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                     recogniton1 = recognition.getLabel();
                  }

                  telemetry.update();
                  if (recogniton1.equals("First")) { //red
                     move(1,1,1,1, (count * 100));
                     autonmousMovement();
                     move(-1,-1, -1, -1, 1000);
                     rest(1000);
                  } else if (recogniton1.equals("Second")) { //blue
                     move(1,1,1,1, (count * 100));
                     autonmousMovement();
                     rest(1000);

                  } else if (recogniton1.equals("Third")) { //green
                     move(1,1,1,1, (count * 100));
                     autonmousMovement();
                     move(1, 1, 1, 1, 750);
                     rest(1000);
                  } else { //if it is not recognized
                     rest(500);
                     move(-1, -1, -1, -1, 100);
                     rest(1000);
                     count++;
                  }
               }
         }
      }
      if(recogniton1.equals("")){
         move(1, 1, 1, 1, (count*100));
         autonmousMovement();
      }
   }
   public void autonmousMovement(){
      //next to cone at start
      //goes to the middle tile between the low and medium junction
      arm("UP", 300);
      rest(200);
      move(0.5,0.5,0.5,0.5, 250);
      rest(500);
      //rotate 90 degrees
      move(1,1,-1, -1,500);
      rest(1000);
      //deliver cone to mid junction

      //maybe move a little forward


      open(500);
      rest(1000);
      move(-1,-1,-1, -1,200);
      arm("DOWN", 500);
      move(-1,-1,-1, -1,300);
      //move back into location 2
      move(1, -1,-1, 1, 1000);
      rest(1000);
   }
   private void initVuforia() {
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
      vuforia = ClassFactory.getInstance().createVuforia(parameters);
   }

   private void initTfod() {
      int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
              "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfodParameters.minResultConfidence = 0.7f;
      tfodParameters.isModelTensorFlow2 = true;
      tfodParameters.inputSize = 300;
      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
   }

   //functions
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

