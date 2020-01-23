package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;
import javax.microedition.khronos.opengles.GL;
import android.media.MediaPlayer;
import android.provider.MediaStore;

import static android.os.SystemClock.sleep;

public class RobotTemplate {
    public RobotTemplate(){

    }
    //VARIABLES AND OBJECTS
    VuforiaSkystone1920 vuforia;
    RevBlinkinLedDriver.BlinkinPattern temp = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    boolean tempFlag1 = false;
    int oldTempInt = 0;
    public double gripServoPos;
    public double errorTemp = 100;
    public boolean booleanTemp =false;

    Number frontLeftMotorPower = 0;
    Number frontRightMotorPower = 0;
    Number rearLeftMotorPower = 0;
    Number rearRightMotorPower = 0;

    Acceleration accel;


    public int initialLiftEncoderCount = 0;
    public double initialGyro = 0;
    
    //Motors go here:
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    DcMotor leftIntake;
    DcMotor rightIntake;

    DcMotor liftMotor;
    DcMotor flipMotor;

    Servo gripServo;
    Servo dragServo;

    Servo capstoneServo;

    Servo leftBlockArm;
    Servo leftBlockArmGrab;

    int oldInt = 0;
    public int tempInt = 0;
    boolean tempBool = false;
int targetPosition = 0;
    double previousErrorTemp;
    double deltaError;


    RevBlinkinLedDriver ledDriver;
   // AnalogInput extEncoder;
    //Sensors go here:
    BNO055IMU imu;

    //ColorSensor color;

    RevTouchSensor LiftTouch;

    ModernRoboticsI2cRangeSensor wallDistance;

    Orientation angles;

    //These hold all the specific methods that the OpModes use
    LinearOpMode autoOpMethods;
    OpMode teleOpMethods;
    /* I'd like a */ Double Cheeseburger; /* with some lettuce and some */ long frenchfries;
    //this is for when autonomous gives "this"
    public RobotTemplate(LinearOpMode mode){
        autoOpMethods = mode;

                // THIS IS AUTONOMOUS
        //initMotorsAuto(leftFront, "leftFront");
        //initMotorsAuto(rightFront, "rightFront");
        //initMotorsAuto(leftRear, "leftRear");
        //initMotorsAuto(rightRear, "rightRear");

        LiftTouch = autoOpMethods.hardwareMap.get(RevTouchSensor.class, "LiftTouch");
        wallDistance = autoOpMethods.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wallDistance");

        leftFront = autoOpMethods.hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = autoOpMethods.hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = autoOpMethods.hardwareMap.get(DcMotor.class, "leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = autoOpMethods.hardwareMap.get(DcMotor.class, "rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = autoOpMethods.hardwareMap.get(DcMotor.class, "leftFrontIntake");
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightIntake = autoOpMethods.hardwareMap.get(DcMotor.class, "rightFrontIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor = autoOpMethods.hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipMotor = autoOpMethods.hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripServo = autoOpMethods.hardwareMap.servo.get("gripServo");

        ledDriver = autoOpMethods.hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

        dragServo = autoOpMethods.hardwareMap.servo.get("dragServo");

        capstoneServo = autoOpMethods.hardwareMap.servo.get("capstoneServo");

        leftBlockArm = autoOpMethods.hardwareMap.servo.get("leftBlockArm");
        leftBlockArmGrab = autoOpMethods.hardwareMap.servo.get("leftBlockArmGrab");

    }

    //and this is for when teleop gives "this"
    public RobotTemplate(OpMode mode){
        teleOpMethods = mode;
            // THIS IS TELEOP
        //initMotorsTele(leftFront, "leftFront");
        //initMotorsTele(rightFront, "rightFront");
        //initMotorsTele(leftRear, "leftRear");
        //initMotorsTele(rightRear, "rightRear");

        LiftTouch = teleOpMethods.hardwareMap.get(RevTouchSensor.class, "LiftTouch");
        wallDistance = teleOpMethods.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wallDistance");

        leftFront = teleOpMethods.hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = teleOpMethods.hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = teleOpMethods.hardwareMap.get(DcMotor.class, "leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = teleOpMethods.hardwareMap.get(DcMotor.class, "rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = teleOpMethods.hardwareMap.get(DcMotor.class, "leftFrontIntake");
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightIntake = teleOpMethods.hardwareMap.get(DcMotor.class, "rightFrontIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor = teleOpMethods.hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flipMotor = teleOpMethods.hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripServo = teleOpMethods.hardwareMap.servo.get("gripServo");

        ledDriver = teleOpMethods.hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

        dragServo = teleOpMethods.hardwareMap.servo.get("dragServo");

        capstoneServo = teleOpMethods.hardwareMap.servo.get("capstoneServo");

    }


    //Specific init stuff for autonomous or teleop can go here, if it can't be done before start
    public void autoInit(){
            autoGyroInit();
            getIntegratedZAxis();
    }

    public void teleInit(){
        teleGyroInit();

        getIntegratedZAxis();
    }

    // AUTONOMOUS METHODS
    // USE autoOpMethods HERE

public double tempAngleVar = 0;

    public void robotDriveFunctions(NickPID pidObject, boolean slow){
        
    if(true) { // Field-Centric-Mecanum

        double r = Math.hypot(autoOpMethods.gamepad1.left_stick_x, autoOpMethods.gamepad1.left_stick_y); // Trig magic
        double robotAngle = Math.atan2(autoOpMethods.gamepad1.left_stick_y, -autoOpMethods.gamepad1.left_stick_x) - Math.PI / 4; // More trigonometry Magic
        autoOpMethods.telemetry.addData("atan thing: ", robotAngle);
        float rightStickX = autoOpMethods.gamepad1.right_stick_x;
        tempAngleVar += rightStickX*11;
        // Just the value of the x-axis of the right stick.
        double rightX = turnRobotTeleop(tempAngleVar, pidObject, 1.4, 1.1);

         double frontLeft = r * Math.cos(robotAngle)+(rightX);
         double frontRight = r * Math.sin(robotAngle)-(rightX);
         double rearLeft = r * Math.sin(robotAngle)+(rightX);
         double rearRight = r * Math.cos(robotAngle)-(rightX);
        if(autoOpMethods.gamepad1.dpad_left){
            rearLeft = 0;
        }else if(autoOpMethods.gamepad1.dpad_right){
            rearRight = 0;
        }

        if(slow){
            frontLeftMotorPower = frontLeft/2;
            frontRightMotorPower = frontRight/2;
            rearLeftMotorPower = rearLeft/2;
            rearRightMotorPower = rearRight/2;
        }else {
            frontLeftMotorPower = frontLeft;
            frontRightMotorPower = frontRight;
            rearLeftMotorPower = rearLeft;
            rearRightMotorPower = rearRight;
        }

    }
    /*else if(autoOpMethods.gamepad1.right_trigger > 0.5) { // Slow Field Centric Mecanum
      double angleOfRobot = integratedZAxis;

      double z = (angleOfRobot%360)*Math.PI/180; // This gives me a value between 0 and 2pi which is linear to the angle of the robot.

      float leftStickX = autoOpMethods.gamepad1.left_stick_x;
      float leftStickY = autoOpMethods.gamepad1.left_stick_y;
      float rightStickX = autoOpMethods.gamepad1.right_stick_x;

      double magnitude = Math.hypot(leftStickX, leftStickY); // Trig magic
      double robotAngle = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4; // More trigonometry Magic
        tempAngleVar += rightStickX*2.5;
        // Just the value of the x-axis of the right stick.
        double rightX = turnRobotTeleop(tempAngleVar, pidObject, 2.5, 1);//initial KP 2.5

        frontLeftMotorPower = (magnitude * Math.cos(robotAngle-z)+(rightX*2))/2;
        frontRightMotorPower = (magnitude * Math.sin(robotAngle-z)-(rightX*2))/2;
        rearLeftMotorPower = (magnitude * Math.sin(robotAngle-z)+(rightX*2))/2;
        rearRightMotorPower = (magnitude * Math.cos(robotAngle-z)-(rightX*2))/2;

    }else if(autoOpMethods.gamepad1.right_bumper && autoOpMethods.gamepad1.left_bumper){ // Robot-Centric Mecanum

      double r = Math.hypot(autoOpMethods.gamepad1.left_stick_x, autoOpMethods.gamepad1.left_stick_y); // Trig magic
      double robotAngle = Math.atan2(autoOpMethods.gamepad1.left_stick_y, -autoOpMethods.gamepad1.left_stick_x) - Math.PI / 4; // More trigonometry Magic
        autoOpMethods.telemetry.addData("atan thing: ", robotAngle);
        float rightStickX = autoOpMethods.gamepad1.right_stick_x;
        tempAngleVar += rightStickX*5;
        // Just the value of the x-axis of the right stick.
        double rightX = turnRobotTeleop(tempAngleVar, pidObject, 1.5, 1.0);

        final double frontLeft = r * Math.cos(robotAngle)+(rightX*2);
      final double frontRight = r * Math.sin(robotAngle)-(rightX*2);
      final double rearLeft = r * Math.sin(robotAngle)+(rightX*2);
      final double rearRight = r * Math.cos(robotAngle)-(rightX*2);

      frontLeftMotorPower = frontLeft;
      frontRightMotorPower = frontRight;
      rearLeftMotorPower = rearLeft;
      rearRightMotorPower = rearRight;

    }else if(autoOpMethods.gamepad1.right_bumper){
      // Slow Robot-Centric Mecanum
      double r = Math.hypot(autoOpMethods.gamepad1.left_stick_x, autoOpMethods.gamepad1.left_stick_y); // Trig magic
      double robotAngle = Math.atan2(autoOpMethods.gamepad1.left_stick_y, -autoOpMethods.gamepad1.left_stick_x) - Math.PI / 4; // More trigonometry Magic

        float rightStickX = autoOpMethods.gamepad1.right_stick_x;
        tempAngleVar += rightStickX*2.5;
        // Just the value of the x-axis of the right stick.
        double rightX = turnRobotTeleop(tempAngleVar, pidObject, 2.5, 1); //initial kP 2.5

      final double frontLeft = (r * Math.cos(robotAngle)+(rightX*2))/2;
      final double frontRight = (r * Math.sin(robotAngle)-(rightX*2))/2;
      final double rearLeft = (r * Math.sin(robotAngle)+(rightX*2))/2;
      final double rearRight = (r * Math.cos(robotAngle)-(rightX*2))/2;

      frontLeftMotorPower = frontLeft;
      frontRightMotorPower = frontRight;
      rearLeftMotorPower = rearLeft;
      rearRightMotorPower = rearRight;

    }else if(autoOpMethods.gamepad1.left_bumper){// Drive like a tank.
        tempAngleVar = -integratedZAxis;
      frontLeftMotorPower = deadZoneReturner(autoOpMethods.gamepad1.left_stick_y);
      rearLeftMotorPower = deadZoneReturner(autoOpMethods.gamepad1.left_stick_y);

      frontRightMotorPower = deadZoneReturner(autoOpMethods.gamepad1.right_stick_y);
      rearRightMotorPower = deadZoneReturner(autoOpMethods.gamepad1.right_stick_y);
    }else{
        tempAngleVar = -integratedZAxis;

        frontLeftMotorPower = deadZoneReturner(autoOpMethods.gamepad1.left_stick_y)/2;
      rearLeftMotorPower = deadZoneReturner(autoOpMethods.gamepad1.left_stick_y)/2;

      frontRightMotorPower = deadZoneReturner(autoOpMethods.gamepad1.right_stick_y)/2;
      rearRightMotorPower = deadZoneReturner(autoOpMethods.gamepad1.right_stick_y)/2;
    }

     */

    assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);

    intake(autoOpMethods.gamepad2.a, autoOpMethods.gamepad2.y, 0.73); // A is take in and Y is push out
   

    
   


//    if(autoOpMethods.gamepad1.a){
//        dragServo.setPosition(0);
//
//    }else{
//        dragServo.setPosition(0.7);
//    }


    }

    public void autoMechanumDriveTime(NickPID pidObject, boolean fieldCentric, double power, double thetaOfTravel, double thetaOfRotation, double timeSeconds){
        double differenceTime = autoOpMethods.time + timeSeconds;
        double startAngle = integratedZAxis;
        while(autoOpMethods.time < differenceTime && autoOpMethods.opModeIsActive()) {
            if (fieldCentric) { // Field-Centric-Mecanum

                double angleOfRobot = integratedZAxis;


                double z = (angleOfRobot % 360) * Math.PI / 180; // This gives me a value between 0 and 2pi which is linear to the angle of the robot.
                double magnitude = power;
                double robotAngle = ((-thetaOfTravel)*Math.PI)/180;
                double rightX = turnRobotTeleopShush( thetaOfRotation - startAngle, pidObject, 2.5, 1);

                frontLeftMotorPower = magnitude * Math.cos(robotAngle - z) + (rightX * 2);
                frontRightMotorPower = magnitude * Math.sin(robotAngle - z) - (rightX * 2);
                rearLeftMotorPower = magnitude * Math.sin(robotAngle - z) + (rightX * 2);
                rearRightMotorPower = magnitude * Math.cos(robotAngle - z) - (rightX * 2);

            } else if (!fieldCentric) { // Robot-Centric Mecanum

                double magnitude = power; // Trig magic
                double robotAngle = ((-thetaOfTravel-90)*Math.PI)/180 - Math.PI / 4; // More trigonometry Magic
                double rightX = turnRobotTeleopShush(thetaOfRotation, pidObject, 1, 0.5);

                final double frontLeft = magnitude * Math.cos(robotAngle) + (rightX );
                final double frontRight = magnitude * Math.sin(robotAngle) - (rightX );
                final double rearLeft = magnitude * Math.sin(robotAngle) + (rightX );
                final double rearRight = magnitude * Math.cos(robotAngle) - (rightX );

                frontLeftMotorPower = frontLeft;
                frontRightMotorPower = frontRight;
                rearLeftMotorPower = rearLeft;
                rearRightMotorPower = rearRight;

            }
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }
    public void autoMechanumSlideTime(NickPID pidObject, boolean fieldCentric, double power, double thetaOfTravel, double thetaOfRotation, double timeSeconds){
        double differenceTime = autoOpMethods.time + timeSeconds;
        double startAngle = integratedZAxis;
        while(autoOpMethods.time < differenceTime && autoOpMethods.opModeIsActive()) {
            if (fieldCentric) { // Field-Centric-Mecanum

                double angleOfRobot = integratedZAxis;


                double z = (angleOfRobot % 360) * Math.PI / 180; // This gives me a value between 0 and 2pi which is linear to the angle of the robot.
                double magnitude = power;
                double robotAngle = ((-thetaOfTravel)*Math.PI)/180;
                double rightX = turnRobotTeleop( thetaOfRotation - startAngle, pidObject, 2.5, 1);

                frontLeftMotorPower = magnitude * Math.cos(robotAngle - z) + (rightX * 2);
                frontRightMotorPower = magnitude * Math.sin(robotAngle - z) - (rightX * 2);
                rearLeftMotorPower = magnitude * Math.sin(robotAngle - z) + (rightX * 2);
                rearRightMotorPower = magnitude * Math.cos(robotAngle - z) - (rightX * 2);

            } else if (!fieldCentric) { // Robot-Centric Mecanum

                double magnitude = power; // Trig magic
                double robotAngle = ((-thetaOfTravel-90)*Math.PI)/180 - Math.PI / 4; // More trigonometry Magic
                double rightX = turnRobotTeleop(thetaOfRotation, pidObject, 1, 0.5);

                final double frontLeft = magnitude * Math.cos(robotAngle) + (rightX );
                final double frontRight = magnitude * Math.sin(robotAngle) - (rightX );
                final double rearLeft = magnitude * Math.sin(robotAngle) + (rightX );
                final double rearRight = magnitude * Math.cos(robotAngle) - (rightX );

                frontLeftMotorPower = frontLeft;
                frontRightMotorPower = frontRight;
                rearLeftMotorPower = rearLeft;
                rearRightMotorPower = rearRight;

            }
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }
    private double P, I, D = 0;
    //private double kP, kI, kD = 0;
    public double previousError = 0;

    public double basicPIDReturnGeneral(double targetValue, double currentValue,  double kP, double kI, double kD, boolean telemetryOption){
        double error = targetValue - currentValue;
        this.P = error;
        this.I += (error*.02);
        this.D = (error - this.previousError);

        double output = (kP * this.P) + (kI * this.I) + (kD*this.D);

        if(false){
            autoOpMethods.telemetry.addData(this.getClass().getName() + "PID Error: ", error);
        }

        if(I > 50){
            I=50;
        }

        if(output > 100){
            output = 100;
        }

        this.previousError = error;
        return output;
    }

    public void autoMechanumDriveEncoder(NickPID pidObject, boolean fieldCentric, double power, double thetaOfTravel, double thetaOfRotation, double inches){
        double startAngle = integratedZAxis;
        double averageEncoder = 0;
        while(Math.abs(averageEncoder) < Math.abs(inches) && autoOpMethods.opModeIsActive()) {
            averageEncoder = ((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2)/2;
            if (fieldCentric) { // Field-Centric-Mecanum


            } else if (!fieldCentric) { // Robot-Centric Mecanum

                double magnitude = -basicPIDReturnGeneral(inches, averageEncoder, 0.2, 0.00, 5, true)/10; // Trig magic
                if(magnitude > 1){
                    magnitude =1;
                }
                if(magnitude < -1){
                    magnitude = -1;
                }
                magnitude = magnitude * power;

                double robotAngle = ((-thetaOfTravel-90)*Math.PI)/180 - Math.PI / 4; // More trigonometry Magic
                double rightX = turnRobotTeleopShush(-(thetaOfRotation + startAngle), pidObject, 1, 0.5);

                final double frontLeft = magnitude * Math.cos(robotAngle) + (rightX );
                final double frontRight = magnitude * Math.sin(robotAngle) - (rightX );
                final double rearLeft = magnitude * Math.sin(robotAngle) + (rightX );
                final double rearRight = magnitude * Math.cos(robotAngle) - (rightX );

                frontLeftMotorPower = frontLeft;
                frontRightMotorPower = frontRight;
                rearLeftMotorPower = rearLeft;
                rearRightMotorPower = rearRight;

            }
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }

    public void autoMechanumDriveEncoderRed(NickPID pidObject, boolean fieldCentric, double power, double thetaOfTravel, double thetaOfRotation, double inches){
        double startAngle = integratedZAxis;
        double averageEncoder = 0;
        while(Math.abs(averageEncoder) < Math.abs(inches) && autoOpMethods.opModeIsActive()) {
            averageEncoder = ((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2)/2;
            if (fieldCentric) { // Field-Centric-Mecanum

                double angleOfRobot = integratedZAxis;


                double magnitude = basicPIDReturnGeneral(inches, averageEncoder, 0.6, 0.08, 2, true);

                if(magnitude > 1){
                    magnitude =1;
                }
                if(magnitude < -1){
                    magnitude = -1;
                }

                magnitude = magnitude * power;
                double robotAngle = ((-thetaOfTravel)*Math.PI)/180;
                double rightX = turnRobotTeleopShush( -(thetaOfRotation + startAngle), pidObject, 2.5, 1);



                frontLeftMotorPower = magnitude * Math.cos(robotAngle) + (rightX * 2);
                frontRightMotorPower = magnitude * Math.sin(robotAngle) - (rightX * 2);
                rearLeftMotorPower = magnitude * Math.sin(robotAngle) + (rightX * 2);
                rearRightMotorPower = magnitude * Math.cos(robotAngle) - (rightX * 2);

            } else if (!fieldCentric) { // Robot-Centric Mecanum

                double magnitude = basicPIDReturnGeneral(inches, averageEncoder, 0.3, 0.08, 2, true); // Trig magic
                if(magnitude > 1){
                    magnitude =1;
                }
                if(magnitude < -1){
                    magnitude = -1;
                }
                magnitude = magnitude * power;

                double robotAngle = ((-thetaOfTravel-90)*Math.PI)/180 - Math.PI / 4; // More trigonometry Magic
                double rightX = turnRobotTeleopShush(-(thetaOfRotation + startAngle), pidObject, 1, 0.5);

                final double frontLeft = magnitude * Math.cos(robotAngle) + (rightX );
                final double frontRight = magnitude * Math.sin(robotAngle) - (rightX );
                final double rearLeft = magnitude * Math.sin(robotAngle) + (rightX );
                final double rearRight = magnitude * Math.cos(robotAngle) - (rightX );

                frontLeftMotorPower = frontLeft;
                frontRightMotorPower = frontRight;
                rearLeftMotorPower = rearLeft;
                rearRightMotorPower = rearRight;

            }
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        assignMotorPowers(0, 0, 0, 0);
        pidObject.resetValues();
        resetEncoderWheels();
    }
    /*
    public void autoMechanumDriveBackLeftSensor(NickPID pidObject, boolean fieldCentric, double power, double thetaOfTravel, double thetaOfRotation, double inches){
        double startAngle = integratedZAxis;
        double sensorValue = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);
        while(Math.abs(sensorValue) > Math.abs(inches) && autoOpMethods.opModeIsActive()) {
            sensorValue = backRightDistanceSensor.getDistance(DistanceUnit.INCH);
            if (fieldCentric) { // Field-Centric-Mecanum

                double angleOfRobot = integratedZAxis;


                double magnitude = -basicPIDReturnGeneral(inches, sensorValue, 0.4, 0, 3, true)/3;

                if(magnitude > 1){
                    magnitude =1;
                }
                if(magnitude < -1){
                    magnitude = -1;
                }

                magnitude = magnitude * power;

                double robotAngle = ((-thetaOfTravel)*Math.PI)/180;
                double rightX = turnRobotTeleopShush( -(thetaOfRotation + startAngle), pidObject, 2.5, 1);



                frontLeftMotorPower = magnitude * Math.cos(robotAngle) + (rightX * 2);
                frontRightMotorPower = magnitude * Math.sin(robotAngle) - (rightX * 2);
                rearLeftMotorPower = magnitude * Math.sin(robotAngle) + (rightX * 2);
                rearRightMotorPower = magnitude * Math.cos(robotAngle) - (rightX * 2);

            } else if (!fieldCentric) { // Robot-Centric Mecanum

                double magnitude = -basicPIDReturnGeneral(inches, sensorValue, 0.4, 0, 0.5, true)/3;
                if(magnitude > 1){
                    magnitude =1;
                }
                if(magnitude < -1){
                    magnitude = -1;
                }
                magnitude = magnitude * power;

                double robotAngle = ((-thetaOfTravel-90)*Math.PI)/180 - Math.PI / 4; // More trigonometry Magic
                double rightX = turnRobotTeleopShush(-(thetaOfRotation + startAngle), pidObject, 1, 0.5);

                final double frontLeft = magnitude * Math.cos(robotAngle) + (rightX );
                final double frontRight = magnitude * Math.sin(robotAngle) - (rightX );
                final double rearLeft = magnitude * Math.sin(robotAngle) + (rightX );
                final double rearRight = magnitude * Math.cos(robotAngle) - (rightX );

                frontLeftMotorPower = frontLeft;
                frontRightMotorPower = frontRight;
                rearLeftMotorPower = rearLeft;
                rearRightMotorPower = rearRight;

            }
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }
    */

    public void autoTurnPID(NickPID pidObject, double targetAngle, double threshold, double kP, double kD){
        while(pidObject.previousError > threshold) {
            double motorPower = turnRobotTeleop(targetAngle, pidObject, kP, kD); // default of 1.5 and 1

            frontLeftMotorPower = motorPower;
            frontRightMotorPower = -motorPower;
            rearLeftMotorPower = motorPower;
            rearRightMotorPower = -motorPower;
            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);

        }
        assignMotorPowers(0, 0, 0, 0);
    }

    public void turnRobotPower(double power){ // Positive gives clockwise motion. direct drive.
        frontLeftMotorPower = power;
        frontRightMotorPower = -power;
        rearLeftMotorPower = power;
        rearRightMotorPower = -power;
        assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);

    }

    public double integral = 0;
    public double previous_error = 0;
    public void turnRobotWithPID(double targetAngle, double pK, double iK, double dK){
        double error = -targetAngle - integratedZAxis;

        double proportional = error;
        integral += (error/.02);
        double derivative = (error - previous_error)/0.02;

        double output = (pK * error) + (iK * integral) + (dK*derivative);

        turnRobotPower(output/10);
        autoOpMethods.telemetry.addData("Error: ", error);
        autoOpMethods.telemetry.update();
        previous_error = error;
    }

    public double getWallDistanceInches(){
        double value = wallDistance.getDistance(DistanceUnit.INCH);
        if(value < 100){
            return value;
        }else{
            return 3;
        }
    }
    
    public void primaryProgram(){// We did this because we don't understand multi-threading
        //autoOpMethods.telemetry.addData("cm", "%.2f cm", wallDistance.getDistance(DistanceUnit.CM));
        //autoOpMethods.telemetry.update();

        if(autoOpMethods.gamepad2.left_stick_button) {
            automatedPickUpTele(autoOpMethods.gamepad2.left_stick_button, 550); // Button and Distance to travel
        }else if(autoOpMethods.gamepad2.right_stick_button) {
            automatedCarriageReturnTele(autoOpMethods.gamepad2.right_stick_button, 0); // Button and safety limit in encoder counts
        }

        liftMoveUp(autoOpMethods.gamepad2.dpad_up, autoOpMethods.gamepad2.dpad_down); // Moves up and down
        flip(autoOpMethods.gamepad2.x, autoOpMethods.gamepad2.b, .45); // Flips the carriage


        if (!autoOpMethods.gamepad2.right_bumper){
            if(autoOpMethods.gamepad2.dpad_left){
                  gripServo.setPosition(0.45); // Open
                  gripServoPos = 0.3;
            }
            if(autoOpMethods.gamepad2.dpad_right){
                gripServo.setPosition(0.3); // Close
                gripServoPos = 0.3;
            }
        } else if (autoOpMethods.gamepad2.right_bumper) {
        if (autoOpMethods.gamepad2.dpad_left && gripServoPos > 0) {
            gripServoPos += .01;
        }
        if (autoOpMethods.gamepad2.dpad_right && gripServoPos < 1) {
            gripServoPos -= .01;
        }
        gripServo.setPosition(gripServoPos);
        }

        if(autoOpMethods.gamepad1.b){
            capstoneServo.setPosition(0.325);
        }else{
            capstoneServo.setPosition(0);
        }

        //if(autoOpMethods.gamepad1.a){
        //    dragServo.setPosition(.825);
        //}else{
          //  dragServo.setPosition(.3);
        //}

        if(autoOpMethods.gamepad1.x){
            leftBlockArm.setPosition(0);
        }else{
            leftBlockArm.setPosition(0.5);
        }

        if(autoOpMethods.gamepad1.y){
            leftBlockArmGrab.setPosition(1);
        }else if(autoOpMethods.gamepad1.a){
            leftBlockArmGrab.setPosition(.4);
        }else {
            leftBlockArmGrab.setPosition(0.0);
        }
    }


    Thread teleOpAutoMatedThread = new Thread(new Runnable() {
      @Override
      public void run(){
       automatedPickUpTele(teleOpMethods.gamepad2.left_stick_button, 550);
       automatedCarriageReturnTele(teleOpMethods.gamepad2.right_stick_button, 75);
       teleOpMethods.telemetry.addData("Hey", "I just met you");
      }
    });

    Thread liftHeightColor = new Thread(new Runnable() {
        @Override
        public void run(){
            while(true) {
                liftHeightAgain(-40, 40, 280); // Adjust this if the lights aren't aligned. offset, size, and frequency
            }
        }
    });

    public void resetIntegratedAxis(){
        integratedZAxis = 0;
        lastHeading = 0;
    }


    public double integratedZAxis = 0;
    public double lastHeading = 0;

    public void getIntegratedZAxis(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;

        double deltaAngle = currentAngle - lastHeading;

        if (deltaAngle < -180)
            deltaAngle += 360 ;
        else if (deltaAngle >= 180)
            deltaAngle -= 360 ;

        integratedZAxis += deltaAngle;

        lastHeading = currentAngle;

        //autoOpMethods.telemetry.update();

    }
    public void telegetIntegratedZAxis(){ // Holy Crap what even is this, I barely understand
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;

        double deltaAngle = currentAngle - lastHeading;

        if (deltaAngle < -180)
            deltaAngle += 360 ;
        else if (deltaAngle >= 180)
            deltaAngle -= 360 ;

        integratedZAxis += deltaAngle;

        lastHeading = currentAngle;

        teleOpMethods.telemetry.addData("integratedZAxis: ", integratedZAxis);
        teleOpMethods.telemetry.update();

    }



    public void autoGyroInit(){
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = autoOpMethods.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void teleGyroInit(){
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = teleOpMethods.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void turnRobotAutonomous(int targetAngle, int sleepTime, NickPID pidObject){ // This is an absolute position system. If you say 90 over and over, it'll not move. If you do 90, 180, 270, etc. itll move around in 90 degree increments
        while(Math.abs(errorTemp) > 3 && autoOpMethods.opModeIsActive()){
            getIntegratedZAxis();
            turnRobotPower(pidObject.basicPIDReturnShush(targetAngle ,1.25,0.1,0.5,false)/100);
        }
        turnRobotPower(0);
        pidObject.resetValues();
        resetEncoderWheels();
        sleep(sleepTime);
        previousErrorTemp = errorTemp;
        errorTemp = 50;
    }
    public void turnRobotAutonomousRed(int targetAngle, int sleepTime, NickPID pidObject){ // This is an absolute position system. If you say 90 over and over, it'll not move. If you do 90, 180, 270, etc. itll move around in 90 degree increments
        while(Math.abs(errorTemp) > 3 && autoOpMethods.opModeIsActive()){
            getIntegratedZAxis();
//            turnRobotPower(pidObject.basicPIDReturnShush(targetAngle ,4,0.05,6,false)/100);
            //turnRobotPower(pidObject.basicPIDReturnShush(targetAngle ,3.8,0.075,8,false)/100);
            turnRobotPower(pidObject.basicPIDReturnShush(targetAngle ,3.5,0.05,8,false)/100);


        }
        turnRobotPower(0);
        pidObject.resetValues();
        resetEncoderWheels();
        sleep(sleepTime);
        errorTemp = 100;
    }
    public double turnRobotTeleop(double targetAngle, NickPID pidObject, double kP, double kD){ // This is an absolute position system. If you say 90 over and over, it'll not move. If you do 90, 180, 270, etc. itll move around in 90 degree increments
           double shush = (pidObject.basicPIDReturn(targetAngle,kP,0.04,kD,false)/100);
        //autoOpMethods.telemetry.addData("Power to turn: ", shush);
        //autoOpMethods.telemetry.update();
            return shush;

    }
    public double turnRobotTeleopShush(double targetAngle, NickPID pidObject, double kP, double kD){ // This is an absolute position system. If you say 90 over and over, it'll not move. If you do 90, 180, 270, etc. itll move around in 90 degree increments
        double shush = (pidObject.basicPIDReturnShush(targetAngle,kP,0.04,kD,false)/100);
        //autoOpMethods.telemetry.addData("Power to turn: ", shush);
        //autoOpMethods.telemetry.update();
        return shush;

    }

    double tempDouble1 = 0;
    public void callAllTelemetry(){
        if(true) {
            autoOpMethods.telemetry.addData("Current Error: ", errorTemp);
            autoOpMethods.telemetry.addData("Average of both: ", Math.abs(((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2))/2);
            autoOpMethods.telemetry.addData("Delta Error ", integratedZAxis);

            //autoOpMethods.telemetry.addData("inches to wall: ", getWallDistanceInches());
            //autoOpMethods.telemetry.addData("Back left and right sensors: ", "{Backleft, BackRight} = %.0f, %.0f", backLeftDistanceSensor.getDistance(DistanceUnit.INCH), backRightDistanceSensor.getDistance(DistanceUnit.INCH) );
            //autoOpMethods.telemetry.addData("Value Shush: ", tempDouble1);

        }
        //imu.getGravity();


        //autoOpMethods.telemetry.addData("Accel: ", accel.xAccel);

        autoOpMethods.telemetry.update();
    }
    public void grabBlock(){
        gripServo.setPosition(0);
    }
    public void releaseBlock(){
        gripServo.setPosition(0.5);
    }

    public float deadZoneReturner ( float y){
        if (y < 0.1 && y > -0.1) {
            return 0;
        } else {
            return y;
        }
    }

    public double mecAdj = 0;
    public double initialAngle = 0;
    public void straightStrafe(double power, double adjFactor, double startingAngle, boolean left){
        initialAngle = startingAngle;
        if (integratedZAxis - startingAngle > 5){
            mecAdj += power / adjFactor;
        }
        if (integratedZAxis - startingAngle < -5){
            mecAdj -= power / adjFactor;
        }
        if (integratedZAxis - startingAngle > -5 && integratedZAxis - startingAngle < 5){
            mecAdj = 0;
        }
        if(left){
            leftFront.setPower(power + mecAdj);
            rightFront.setPower(-power + mecAdj);
            leftRear.setPower(-power + mecAdj);
            rightRear.setPower(power + mecAdj);
        }
        if(!left){
            leftFront.setPower(-power + mecAdj);
            rightFront.setPower(power + mecAdj);
            leftRear.setPower(power + mecAdj);
            rightRear.setPower(-power + mecAdj);
        }
    }

    public void autoStrafeTime(NickPID pidObject, double power, double angleofTravel, double angleofRotation, double timeSeconds){
        double r = power;
        double startAngle = integratedZAxis;
        double differenceTime = autoOpMethods.time + timeSeconds;


        double robotAngle = Math.atan2(Math.sin(((angleofTravel-90)*Math.PI)/180), -Math.cos(((angleofTravel-90)*Math.PI)/180)) - Math.PI / 4; // More trigonometry Magic

        while(autoOpMethods.time < differenceTime && autoOpMethods.opModeIsActive()) {
            // Just the value of the x-axis of the right stick.
            double rightX = turnRobotTeleop(angleofRotation - startAngle, pidObject, 2.5, 1);

            final double frontLeft = r * Math.cos(robotAngle) + (rightX * 2);
            final double frontRight = r * Math.sin(robotAngle) - (rightX * 2);
            final double rearLeft = r * Math.sin(robotAngle) + (rightX * 2);
            final double rearRight = r * Math.cos(robotAngle) - (rightX * 2);

            frontLeftMotorPower = frontLeft;
            frontRightMotorPower = frontRight;
            rearLeftMotorPower = rearLeft;
            rearRightMotorPower = rearRight;

            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }
    public void autoStrafeDistanceSensorLeftorRight(NickPID pidObject, double power, String leftOrright, double distanceInches){
        double angleofTravel = 0;

        if(leftOrright == "left" || leftOrright == "Left"){
            angleofTravel = 90;
        }else if(leftOrright == "right" || leftOrright == "Right"){
            angleofTravel = -90;
        }

        double distanceSensorValue = getWallDistanceInches();
        double error = distanceSensorValue - distanceInches;

        double r = basicPIDReturnGeneral(distanceSensorValue, distanceInches, 0.1, 0.85, 1.75, false);
        //double r = basicPIDReturnGeneral(distanceSensorValue, distanceInches, 0.1, 1.0, 1.75, false);



        if(r > 1){
            r =1;
        }
        if(r < -1){
            r = -1;
        }
        r= r*power;
        double startAngle = integratedZAxis;

        double robotAngle = Math.atan2(Math.sin(((angleofTravel-90)*Math.PI)/180), -Math.cos(((angleofTravel-90)*Math.PI)/180)) - Math.PI / 4; // More trigonometry Magic

        resetEncoderWheels();

        double startingEncoder = ((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2)/2;
        double driftOffset = 0;

        while( Math.abs(error) > 2 && autoOpMethods.opModeIsActive()) {
            distanceSensorValue = getWallDistanceInches();
            error = distanceSensorValue - distanceInches;
            // Just the value of the x-axis of the right stick.
            double rightX = turnRobotTeleopShush(-startAngle, pidObject, 2.5, 1);

//            driftOffset = basicPIDReturnGeneral((((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2)/2), startingEncoder, 0.6, 0.08, 1.75, false);
            driftOffset = basicPIDReturnGeneral((((((8*Math.PI)/2400)*leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*rightFront.getCurrentPosition()/2)/2), startingEncoder, 0.6, 1.0, 1.75, false);

            if(driftOffset > 1){
                driftOffset =1;
            }
            if(driftOffset < -1){
                driftOffset = -1;
            }
            final double frontLeft = r * Math.cos(robotAngle) + (rightX ) + driftOffset/8;
            final double frontRight = r * Math.sin(robotAngle) - (rightX ) + driftOffset/8;
            final double rearLeft = r * Math.sin(robotAngle) + (rightX ) + driftOffset/8;
            final double rearRight = r * Math.cos(robotAngle) - (rightX ) + driftOffset/8;

            frontLeftMotorPower = frontLeft;
            frontRightMotorPower = frontRight;
            rearLeftMotorPower = rearLeft;
            rearRightMotorPower = rearRight;

            autoOpMethods.telemetry.addData("front L: ", frontLeftMotorPower);
            autoOpMethods.telemetry.addData("front R: ", frontRightMotorPower);
            autoOpMethods.telemetry.addData("rear L: ", rearLeftMotorPower);
            autoOpMethods.telemetry.addData("rear R: ", rearRightMotorPower);

            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }
    public void autoStrafeDistanceSensor(NickPID pidObject, double power, double angleofTravel, double angleofRotation, double distanceInches){

        double distanceSensorValue = getWallDistanceInches();
        double error = distanceSensorValue - distanceInches;

        //double r = basicPIDReturnGeneral(distanceSensorValue, distanceInches, 0.6, 0.08, 1.75, false)*power;

        double r = basicPIDReturnGeneral(distanceSensorValue, distanceInches, 0.6, 1, 1.75, false)*power;


        if(r > 1){
            r =1;
        }
        if(r < -1){
            r = -1;
        }

        double startAngle = integratedZAxis;

        double robotAngle = Math.atan2(Math.sin(((angleofTravel-90)*Math.PI)/180), -Math.cos(((angleofTravel-90)*Math.PI)/180)) - Math.PI / 4; // More trigonometry Magic

        while( Math.abs(error) > 2 && autoOpMethods.opModeIsActive()) {
            distanceSensorValue = getWallDistanceInches();
            error = distanceSensorValue - distanceInches;
            // Just the value of the x-axis of the right stick.
            double rightX = turnRobotTeleop(angleofRotation - startAngle, pidObject, 2.5, 1);

            final double frontLeft = r * Math.cos(robotAngle) + (rightX * 2);
            final double frontRight = r * Math.sin(robotAngle) - (rightX * 2);
            final double rearLeft = r * Math.sin(robotAngle) + (rightX * 2);
            final double rearRight = r * Math.cos(robotAngle) - (rightX * 2);

            frontLeftMotorPower = frontLeft;
            frontRightMotorPower = frontRight;
            rearLeftMotorPower = rearLeft;
            rearRightMotorPower = rearRight;

            assignMotorPowers(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower);
        }
        pidObject.resetValues();
        resetEncoderWheels();
        assignMotorPowers(0, 0, 0, 0);
    }



    public void resetEncoderWheels(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void stopDriveMotors(){
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }
    public int getBlockSwitchCase(double blockPosition, double rangeDifference){
        if(blockPosition > rangeDifference){
            return 1;
        }else if(blockPosition < -rangeDifference){
            return 2;
        } else{
            return 3;
        }

    }

    public void assignMotorPowers(Number frontLeft, Number frontRight, Number rearLeft, Number rearRight){
        leftFront.setPower(frontLeft.floatValue());
        rightRear.setPower(rearRight.floatValue());
        leftRear.setPower(rearLeft.floatValue());
        rightFront.setPower(frontRight.floatValue());
    }


    public void intake(boolean button, boolean reverseButton, double power){
        if(button){
            leftIntake.setPower(power);
            rightIntake.setPower(power);
        } else if(reverseButton) {
            leftIntake.setPower(-power);
            rightIntake.setPower(-power);
        }else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void liftMoveUp(boolean button, boolean button2){
        if(button){
            liftMotor.setPower(1);
        } else if(button2){
            liftMotor.setPower(-1);
        } else {
            liftMotor.setPower(0);
        }
    }

    public void flip(boolean button, boolean button2, double power){
        if(button){
            flipMotor.setPower(power);
        } else if(button2){
            flipMotor.setPower(-power);
        } else {
            flipMotor.setPower(0);
        }
    }


    public void backupToPlate(double inchesAway, double power, double timeSeconds){
/*

        double startTime = autoOpMethods.time;

        double DistanceSensorValue = 0;
        if(backLeftDistanceSensor.getDistance(DistanceUnit.INCH) < 100){
            DistanceSensorValue = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);
        }else if(backRightDistanceSensor.getDistance(DistanceUnit.INCH) < 100){
            DistanceSensorValue = backRightDistanceSensor.getDistance(DistanceUnit.INCH);
        }else{
            DistanceSensorValue = 10;
            timeSeconds = .7;
        }

        while ((DistanceSensorValue > inchesAway) && (autoOpMethods.time < startTime + timeSeconds) && autoOpMethods.opModeIsActive()){
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);


        }
        stopDriveMotors();

 */
    }



    public void automatedPickUpTele( boolean button, int newPosition){
        if(button) {
            int targetPosition = initialLiftEncoderCount + newPosition;
            gripServo.setPosition(0.3);
            sleep(300);
            while((liftMotor.getCurrentPosition() < targetPosition)){
                liftMotor.setPower(1);
            }
            liftMotor.setPower(0);
            sleep(300);
        }}

    double timeObject = 0;
    boolean timeFlag = false;
    public boolean basicInlineTimer(int timeDelay){
        if(!timeFlag){
            timeObject = teleOpMethods.time + timeDelay;
            timeFlag = true;
        }else if(timeFlag && autoOpMethods.time < timeObject){

        }else if(timeFlag && autoOpMethods.time > timeObject) {

            timeFlag = false;
            return true;
        }
        return false;
    } 
    

    
    public void automatedCarriageReturnTele(boolean button, int safetyStop){
        if(button) {
            
            flipMotor.setPower(-0.45);
            sleep(500);
            flipMotor.setPower(0);
            
            gripServo.setPosition(0.3);
            sleep(300);
            while (!(LiftTouch.isPressed()) && liftMotor.getCurrentPosition() > initialLiftEncoderCount - safetyStop) {
                if (liftMotor.getCurrentPosition() >= (initialLiftEncoderCount + 300)) {
                    liftMotor.setPower(-1);
                } else {
                    liftMotor.setPower(-.5);
                }
            }
            liftMotor.setPower(0);
            gripServo.setPosition(0.45);

        }
    }

    public int liftLevel(int offset, int targetRange, int frequency) { //shush dingus
        int currentEncoder = liftMotor.getCurrentPosition();
        int tempInt = 0;

        for (int i = 0; i < 10; i++) {
            if ((currentEncoder > (((frequency * i) - targetRange)+offset)) && (currentEncoder < (((frequency * i) + targetRange)+offset))) {
                tempInt = i;
                break;
            } else {
                tempInt = -1;
            }
        }
        return tempInt;
    }
    public void liftHeightAgain(int offset, int targetRange, int frequency){
        int level = liftLevel(offset, targetRange, frequency);

        if(level>=1){
            int difference = level - oldTempInt;

            if(difference > 0){
                temp = temp.next();
            }else if(difference < 0){
                temp = temp.previous();
            }
            ledDriver.setPattern(temp);
            oldTempInt = level;
        }else if(level == -1){
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }
    }



}
