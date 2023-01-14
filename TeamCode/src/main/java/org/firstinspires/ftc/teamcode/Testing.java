
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

@Autonomous(name = "Testing")
public class Testing extends LinearOpMode {


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
        waitForStart();
                close(500);
                arm("UP", 1000);
               move(1, 1,1,1,200);
               rest(200);
               move(1,1,-1,-1,1000);
               rest(500);
               arm("UP", 500);
               move(1,1, -1,-1,1000);
        rest(500);
        move(1, 1,1,1,500);
        rest(500);
        move(-1,-1,1,1,260);
        move(1,1,1,1, 250);
        rest(500);
        arm("UP", 500);
        open(500);



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
