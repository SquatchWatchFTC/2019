package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Red Full Autonomous", group = "ready")
//@Disabled
public class RedFullAutonomous extends LinearOpMode {

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

    servoMove startServoMove = new servoMove(1, false,0);
    servoMove servoMove1stPassBack;
    servoMove servoMove2ndPass;
    servoMove servoMove1stPass;
    robot.resetEncoderWheels();

    robot.initVoltage = robot.getBatteryVoltage();

    imuUpdater.start();

    telemetry.addData("Robot Status:", "Initialized.");
    telemetry.update();
//    robot.storeRightArm();
//    robot.storeLeftArm();
    robot.rightBlockArm.setPosition(0.75);
    robot.rightBlockArmGrab.setPosition(1);
    robot.leftBlockArm.setPosition(0.75);
    robot.leftBlockArmGrab.setPosition(0.5);

    double power =(((-0.2*1.01)*robot.initVoltage)+3.5);
    double deriv = 0;

    waitForStart();
    startServoMove.start();
    robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, -26.1 , power/1.4, 0.0, 10,true); // 0.3 and 0.3 for short distances
    robot.turnRobotAutonomous(90, 0, turnPID,power,0.0, 2);
    robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -3, 0.3, 0, true); // 0.3 and 0.3 for short distances\

    if(((double)robot.rightColor.red()/(double)robot.rightColor.green())<0.8){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -1, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 2; foundBlock = true;
    }else{
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 8, 0.4, 0, true); // 0.3 and 0.3 for short distances\
    }
    //sleep(500);
    if(((double)robot.rightColor.red()/(double)robot.rightColor.green())<0.8 && !foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, -1, 0.3, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 3; foundBlock = true;
    }else if (!foundBlock){
      robot.autonomousNewMechDrive(turnPID, 1, 0, 0, 7, 0.4, 0, true); // 0.3 and 0.3 for short distances\
      blockPos = 1;
    }

      robot.rightReadyToGrabExtend();
      sleep(250);
      robot.rightReadyToDrop();
      sleep(500);
      robot.driveRightArm();

      if(blockPos == 1){
        servoMove1stPass = new servoMove(65, true,0);
      }else if(blockPos ==2){
         servoMove1stPass = new servoMove(54, true,0);
      }else {
        servoMove1stPass = new servoMove(46, true,0);
      }

    servoMove1stPass.start();
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
      servoMove1stPassBack  = new servoMove(104, false,100);
    }else {
      servoMove1stPassBack  = new servoMove(111, false,0);
    }

    servoMove1stPassBack.start();
    if(blockPos == 1){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 71, 1.3, 0,40, true); // 0.3 and 0.3 for short distances\
    }else if( blockPos == 2){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 104, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }else if(blockPos == 3){
      robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 112, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
    }
    //robot.autoMechanumSlideTime(turnPID, false,1,90, 0,0.2);
    //robot.turnRobotAutonomous(-2, 0, turnPID, .02, 0);

    robot.turnRobotAutonomous(80, 0, turnPID,power/6,deriv,6);

    robot.turnRobotAutonomous(90, 0, turnPID,power/6,deriv,2);

    robot.rightReadyToGrabExtend();
    sleep(250);
    robot.rightReadyToDrop();
    sleep(500);
    robot.driveRightArm();

    servoMove2ndPass = new servoMove(-1, true,0);

    if(blockPos == 1){
      robot.autonomousNewMechDriveGradualLoose(turnPID, 1, 0, 0, -71, 1.3, 0, 40,true); // 0.3 and 0.3 for short distances\
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

    sleep(250);
    while(robot.integratedZAxis > -92 && opModeIsActive()){
      robot.assignMotorPowers(-1,1,-1,0);

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
            robot.rightDrop();
            try {
              Thread.sleep(750);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.rightDropOpen();
            try {
              Thread.sleep(350);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.driveRightArm();
          }else{
            try {
              Thread.sleep(SLEEPMS);
            } catch(InterruptedException e) {
              System.out.println("got interrupted!: " + e);
            }
            robot.rightReadyToGrab();
          }
          thing = true;
        }
        idle();
      }
    }
  }


}
