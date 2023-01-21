
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

@Autonomous(name = "rightSideAuto")
public class rightSideAuto extends LinearOpMode {


    //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
    //expansion hub: arm = 0, arm2 is 1
    public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;

    public CRServo leftSide, rightSide, flipper, claw;

    public DistanceSensor backDistanceSensor, leftDistanceSensor, rightDistanceSensor;
    //the variables that keep track of movement
    double armSpeed, turnSpeed;
    //armExtension Value

    //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
    double[] wheels = new double[4];
    public int leftFrontPos;
    public int rightFrontPos;
    public int leftBackPos;
    public int rightBackPos;
    public int armLeftPos;
    public int armRightPos;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/powerPlayModel.tflite";


    private static final String[] LABELS = {
            "Number One",
            "Number Two",
            "Number Three"
    };

    private static final String VUFORIA_KEY =
            "AY+LAO//////AAABmXqpfGFoHkhWrS1QLc/3RBMitc3sWaySydW66iqgCgULFQBcw/vHIj1o9YeqLzsHNYgoJ4bigISHSEJ0aqFhiZ1r+rRJ0HhFgL4V88oT/6FlJFzRDwhtVX+72HEoEIZgiH2vZD5i5mQp11U9rz+oE/07CwUzmABaW9i4gI50lPJhve1K6xRd4ydgjVhdiJ/Ayz6X8mK+LBnX10KxS0cCEtbCi2texH/X9W+iMXoXk9bzfnKi8X0xJxOAO9F5R2Ja9wBvEZrQQFhf/e4wegFY3fTYqsKtRTaENLYtQFuPqelbRuFRR2qcIZ9Q677IXEM+Ydagzu1bj2lBc6ueZf5SYn0iaPOIWRp4ilLNlRj5wyZ4";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

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
        claw = hardwareMap.get(CRServo.class, "claw");
        flipper = hardwareMap.get(CRServo.class, "flipper");
        leftSide = hardwareMap.get(CRServo.class, "leftSide");
        rightSide = hardwareMap.get(CRServo.class, "rightSide");

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
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;
        armLeftPos = 0;
        armRightPos = 0;
        waitForStart();
        //Get into position to aim the camera
        close(100);
        flipUp();
        arm("UP", 550);

        forward(1, 500);


        rest(500);
        //tensorflow
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // put the code that goes before the checking here
        //close the arm to grab the cone and then pick it up and then move forward to the cone and then do a 180 rotation to scan

        String recogniton1 = "";
        int count = 0;
        while (recogniton1.equals("") && count < 4) {
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
                    if (recogniton1.equals("Number One")) { //red

                        autonmousMovement();
                       left(1,1000);
                        rest(1000);
                    } else if (recogniton1.equals("Number Two")) { //blue

                        autonmousMovement();
                        rest(1000);

                    } else if (recogniton1.equals("Number three ")) { //green

                        autonmousMovement();
                       right(1,1000);
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
        if (recogniton1.equals("")) {

            autonmousMovement();
        }

        //Move to the targetted parking space

    }

    public void autonmousMovement(){
        //next to cone at start
        //goes to the middle tile between the low and medium junction
        //Move the robot to the high junctions and then drop a cone
        flip();
        rest(500);


        forward(1, 5400);
        rest(500);
        arm("UP", 1000);
        turnLeft(1, 2000);
        forward(1, 1000);
        flipDown();
        open(100);
        rest(500);
        //Center the robot on the mat to then go and pick up a new cone
        backward(1, 1000);
        turnRight(1, 1500);
        rightNine();
        forward(1, 2500);

        arm("DOWN", 1000);
        rest(500);
        close(100);
        rest(500);
        //Move the robot back to the high junction
        backward(1, 2500);
        leftNine();
        turnLeft(1, 2000);
        forward(1, 1000);
        //Move to the targetted parking space


        //Move the robot to the high junctions and then drop a cone

        flip();

        rest(500);


        forward(1, 5400);

        rest(500);

        arm("UP", 1000);

        turnLeft(1, 2000);

        forward(1, 1000);

        flipDown();

        open(100);

        rest(500);

        //Center the robot on the mat to then go and pick up a new cone
        backward(1, 1000);

        turnRight(1, 1500);

        rightNine();

        forward(1, 2500);

        arm("DOWN", 1000);

        rest(500);

        close(100);

        rest(500);

        //Move the robot back to the high junction
        backward(1, 2500);

        leftNine();

        turnLeft(1, 2000);

        forward(1, 1000);
        backward(1,1000);
        turnRight(1,2000);
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
    public void flipUp(){
        rightSide.setPower(1);
        leftSide.setPower(-1);
        flipper.setPower(1);
        sleep(100);
        rightSide.setPower(0);
        leftSide.setPower(0);
        flipper.setPower(0);
    }
    public void flipDown(){
        rightSide.setPower(-1);
        leftSide.setPower(1);
        flipper.setPower(-1);
        sleep(100);
        rightSide.setPower(0);
        leftSide.setPower(0);
        flipper.setPower(0);
    }
    public void open(int duration){
        claw.setPower(-1);

        sleep(duration);
    }
    public void close(int duration){
        claw.setPower(1);

        sleep(duration);
    }
    public void move(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, int target) {
        leftFrontPos += target*frontLeftPower;
        rightFrontPos += target*frontRightPower;
        leftBackPos += target*backLeftPower;
        rightBackPos += target*backRightPower;


        //Setting the target positions
        frontLeftWheel.setTargetPosition(leftFrontPos);
        frontRightWheel.setTargetPosition(rightFrontPos);
        backLeftWheel.setTargetPosition(leftBackPos);
        backRightWheel.setTargetPosition(rightBackPos);
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
        move(-1,-1,1,1, 5100);
    }
    public void rightNine(){
        move(1,1,-1,-1, 2550);
    }
    public void leftNine(){
        move(-1,-1,1,1, 2550);
    }
    public void continuousMove(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftWheel.setPower(frontLeftPower);
        backLeftWheel.setPower(backLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backRightWheel.setPower(backRightPower);
    }
    public void arm(String direction, int target) {
        if (direction.equals("UP")) {
            armRightPos += target;
            armLeftPos-=target;
            arm.setTargetPosition(armRightPos);
            arm2.setTargetPosition(armLeftPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            arm2.setPower(1);
           /* while(opModeIsActive() && armActive()){
                idle();
            }

            */
        }
        else if (direction.equals("DOWN")) {
            armRightPos -= target;
            armLeftPos += target;
            arm.setTargetPosition(armRightPos);
            arm2.setTargetPosition(armLeftPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            arm2.setPower(1);
            /*while(opModeIsActive() && armActive()){
                idle();
            }

             */
        }

    }
   /* public boolean armActive(){
        if(arm.isBusy() && arm2.isBusy()){
            return true;
        }else{
            return false;
        }
    }

    */



    public void rest(int duration) {
        frontLeftWheel.setPower(0);
        backLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backRightWheel.setPower(0);
        sleep(duration);
    }
}
