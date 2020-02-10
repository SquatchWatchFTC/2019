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


@Autonomous(name = "Blue Build-Plate Autonomous", group = "ready")
//@Disabled
public class BlueBuildPlate extends LinearOpMode {

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

    robot.resetEncoderWheels();

    imuUpdater.start();

    telemetry.addData("Robot Status:", "Initialized.");
    telemetry.update();

    double robotDriveToBlock = Math.abs(robot.getBlockDistanceInches() -3.25);
    double robotDriveToWall = robot.getWallDistanceInches() - 3.3;

    waitForStart();

    robot.readyToGrab();
    sleep(500);
    robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -26.7 , 0.3, 0.0, true); // 0.3 and 0.3 for short distances
    robot.turnRobotAutonomous(-90, 1000, turnPID,0.35,0.0);
    sleep(250);
    robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -3, 0.3, 0, true); // 0.3 and 0.3 for short distances\
    sleep(250);

    if(((double)robot.leftColor.red()/(double)robot.leftColor.green())<0.7){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -4, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 2; foundBlock = true;
    }else{
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 8, 0.3, 0, true); // 0.3 and 0.3 for short distances\
    }
    sleep(500);
    if(((double)robot.leftColor.red()/(double)robot.leftColor.green())<0.7 && !foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -4, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 3; foundBlock = true;
    }else if (!foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -19, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 1;
    }
    sleep(500);


      robot.grabLeftArm();
      sleep(750);
      robot.driveLeftArm();
      robot.turnRobotAutonomous(-90, 250, turnPID, 0.35, 0);
      sleep(500);

    if(blockPos == 1){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -79, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if( blockPos == 2){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -87, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if(blockPos == 3){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -95, 1, 0, true); // 0.3 and 0.3 for short distances\

    }

    sleep(500);

    robot.readyToDrop();
    sleep(500);
    robot.readyToGrab();

    robot.driveLeftArm();
    robot.turnRobotAutonomous(-90, 250, turnPID,0.35,0.0);

    if(blockPos == 1){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 103.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if( blockPos == 2){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 111.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if(blockPos == 3){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 119.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }

    robot.readyToGrab();
    sleep(1000);

    //robot.autoMechanumSlideTime(turnPID, false,0.4,-90, 0,1.5);

    robot.turnRobotAutonomous(-90, 250, turnPID, 0.35, 0);

    robot.grabLeftArm();
    sleep(750);
    robot.driveLeftArm();
    robot.turnRobotAutonomous(-90, 250, turnPID, 0.35, 0);
    sleep(500);

    if(blockPos == 1){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -103.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if( blockPos == 2){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -111.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }else if(blockPos == 3){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -119.5, 1, 0, true); // 0.3 and 0.3 for short distances\

    }


    sleep(500);

    robot.readyToDrop();
    sleep(1000);
    robot.readyToGrab();

    robot.driveLeftArm();
    while(true && robot.autoOpMethods.opModeIsActive()){

    }

    //robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 18);

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


  private class liftThing extends Thread
  {
    boolean pickupFlag = false;
    public liftThing(boolean pickup)
    {
      pickupFlag = pickup;
      this.setName("liftThingThread");
    }


    @Override
    public void run()
    {
      boolean thing = true;

      while (!isInterrupted())
      {
        if(pickupFlag && thing) {
          robot.automatedPickUpTele(thing, 350);
          try {
            Thread.sleep(500);
          } catch(InterruptedException e) {
            System.out.println("got interrupted!: " + e);
          }
          robot.flip(true, false, .45);
          try {
            Thread.sleep(700);
          } catch(InterruptedException e) {
            System.out.println("got interrupted!: " + e);
          }
          robot.flip(false, false, .45);
          thing = false;
        }else if(!pickupFlag && thing){
          robot.flipMotor.setPower(-0.45);
          try {
            Thread.sleep(700);
          } catch(InterruptedException e) {
            System.out.println("got interrupted!: " + e);
          }
          robot.flipMotor.setPower(0);

          robot.automatedCarriageReturnTele(thing, 0);
          thing = false;
        }
        idle();
      }
    }
  }


}
