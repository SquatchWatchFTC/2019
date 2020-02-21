package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;
import com.vuforia.CameraDevice;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.tfod.Timer;


@Autonomous(name = "Blue Full Autonomous", group = "ready")
//@Disabled
public class BlueFullAutonomous extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private RobotTemplate robot;
  NickPID turnPID; // For the love of god don't forget to feed in robot object
  private int blockPos = 0;
  private boolean foundBlock = false;




  @Override
  public void runOpMode(){
    robot = new RobotTemplate(this);
    telemetry.addData("Robot Status:", "Initializing, Please Wait.");
    telemetry.update();

    turnPID = new NickPID(robot);

    robot.autoInit();

    integrationOfAxis imuUpdater = new integrationOfAxis();

    servoMove startServoMove = new servoMove(1, false, 0);
    servoMove servoMove1stPassBack;
    servoMove servoMove2ndPass;
    servoMove servoMove1stPass;
    servoMove servoLastFunkyTurn = new servoMove(-1, false,0);
    robot.resetEncoderWheels();

    robot.initVoltage = robot.getBatteryVoltage();

    imuUpdater.start();

    telemetry.addData("Robot Status:", "Initialized.");
    telemetry.update();

    robot.rightBlockArm.setPosition(0.75);
    robot.rightBlockArmGrab.setPosition(1);
    robot.leftBlockArm.setPosition(0.75);
    robot.leftBlockArmGrab.setPosition(0.5);
//    robot.storeRightArm();
//    robot.storeLeftArm();

    double power =(((-0.2*1.01)*robot.initVoltage)+3.5);
    double deriv = 0;


    waitForStart();

    startServoMove.start();
    robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, -25.8 , power/1.4, 0.0, 10,true); // 0.3 and 0.3 for short distances
    robot.turnRobotAutonomous(-90, 0, turnPID,power/1.5,deriv,2);
    robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -3, 0.3, 0, true); // 0.3 and 0.3 for short distances\

    if(((double)robot.leftColor.red()/(double)robot.leftColor.green())<0.7){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -4, 0.4, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 2; foundBlock = true;
    }else{
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 8, 0.4, 0, true); // 0.3 and 0.3 for short distances\
    }
    //sleep(500);
    if(((double)robot.leftColor.red()/(double)robot.leftColor.green())<0.7 && !foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -4, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 3; foundBlock = true;
    }else if (!foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 5, 0.4, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 1;
    }
    //sleep(500);
      robot.leftReadyToGrabExtend();
      sleep(250);
      robot.leftReadyToDrop();
      sleep(500);
      robot.driveLeftArm();
      if(blockPos == 1){
        servoMove1stPass = new servoMove(65, true,0);
      }else if(blockPos ==2){
         servoMove1stPass = new servoMove(54, true,0);
      }else {
        servoMove1stPass = new servoMove(46, true,0);
      }


    servoMove1stPass.start();// First drive up with block, primes arm to  go out and drop block
    if(blockPos == 1){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -95, 1.3, 0,40, true); // 0.3 and 0.3 for short distances\
    }else if( blockPos == 2){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -79, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }else if(blockPos == 3){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -87, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }

    if(blockPos == 1){
      servoMove1stPassBack = new servoMove(70, false,100);
    }else if(blockPos == 2){
      servoMove1stPassBack  = new servoMove(105, false,100);
    }else {
      servoMove1stPassBack  = new servoMove(113, false,100);
    }

    servoMove1stPassBack.start(); // After placing 1st block, returning to get the second, primes to put arm out right when it gets to the block
    if(blockPos == 1){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 71, 1.3, 0,40, true); // 0.3 and 0.3 for short distances\
    }else if( blockPos == 2){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 106, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }else if(blockPos == 3){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 114, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }
    //robot.autoMechanumSlideTime(turnPID, false,1,90, 0,0.2);
    //robot.turnRobotAutonomous(-2, 0, turnPID, .02, 0);

    robot.turnRobotAutonomous(-80, 0, turnPID,power/6,deriv,3);

    robot.turnRobotAutonomous(-91, 0, turnPID,power/6,deriv,2);// Butt wiggle to put arm down

    robot.leftReadyToGrabExtend();
    sleep(250);
    robot.leftReadyToDrop(); // 2nd block pickup
    sleep(500);
    robot.driveLeftArm();

    servoMove2ndPass = new servoMove(-1, true, 0);

    if(blockPos == 1){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -69, 1.7, 0, 40,true); // 0.3 and 0.3 for short distances\
    }else if( blockPos == 2){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -100, 1.7, 0, 40, true); // 0.3 and 0.3 for short distances\
    }else if(blockPos == 3){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -110.5, 1.7, 0, 40, true); // 0.3 and 0.3 for short distances\
    }



    servoMove1stPass.interrupt();
    servoMove1stPassBack.interrupt();
    startServoMove.interrupt();

    robot.turnRobotAutonomous(0, 0, turnPID, power,deriv,6);

    while(robot.blockDistance.getDistance(DistanceUnit.INCH) >= 1 && opModeIsActive()){
      robot.assignMotorPowers(0.2,0.2,0.2,0.2);
    }   robot.assignMotorPowers(0,0,0,0);
    robot.dragServo.setPosition(1);
    sleep(750);
    while(Math.abs(((((8*Math.PI)/2400)*robot.leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*robot.rightFront.getCurrentPosition()/2)/2) < 13 && opModeIsActive()){
      robot.assignMotorPowers(-1,-1,-1,-1);
    }
    robot.assignMotorPowers(0, 0, 0, 0);
    robot.storeLeftArmTele();
    robot.storeRightArmTele();
    sleep(250);
    while(robot.integratedZAxis < 88 && opModeIsActive()){
      robot.assignMotorPowers(1,-1,0,-1);

    }
    servoMove2ndPass.start();

    robot.dragServo.setPosition(0.25);
    robot.assignMotorPowers(1,1,0,0);
    sleep(750);
    robot.assignMotorPowers(0, 0, 0, 0);


    robot.resetEncoderWheels();

    robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 38, 0.6, 0, true); // 0.3 and 0.3 for short distances\

    imuUpdater.interrupt();

  }




  private class integrationOfAxis extends Thread
  {
    public integrationOfAxis()
    {
      this.setName("integratedZAxisThread");
    }

    @Override
    public void run()
    {
      while (!isInterrupted())
      {
        robot.getIntegratedZAxis();
        robot.callAllTelemetry(turnPID); // Do not call telem.update at ALL. It'll freak out.

        idle();
      }
    }
  }




  private class servoMove extends Thread
  {
    double distance;
    int SLEEPMS;
    boolean grab;
    public servoMove(double distanceTilActivation, boolean grabBlock, int sleepMs)
    {
      this.setName("servoThing");
      distance = distanceTilActivation;
      grab = grabBlock;
      SLEEPMS = sleepMs;
    }


    @Override
    public void run()
    {
      boolean thing = false;

      while (!isInterrupted() && !thing)
      {
        if(Math.abs(((((8*Math.PI)/2400)*robot.leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*robot.rightFront.getCurrentPosition()/2)/2) > (distance)){
          if(grab){
            robot.leftDrop();
            try {
              Thread.sleep(750);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.leftDropOpen();
            try {
              Thread.sleep(350);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.driveLeftArm();

          }else{
            try {
              Thread.sleep(SLEEPMS);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.leftReadyToGrab();
          }
          thing = true;
        }
        idle();
      }
    }
  }


}
