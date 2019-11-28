package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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


@Autonomous(name = "Autonomous 1", group = "test")
//@Disabled
public class Autonomous1 extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private RobotTemplate robot;




  @Override
  public void runOpMode(){
    robot = new RobotTemplate(this);
    telemetry.addData("Robot Status:", "Initializing, Please Wait.");
    telemetry.update();

    robot.autoInit();


    NickPID turnPID = new NickPID(robot); // For the love of god don't forget to feed in robot object
    robot.vuforia = new VuforiaSkystone1920();
    integrationOfAxis imuUpdater = new integrationOfAxis();
    robot.resetEncoderWheels();

      robot.vuforia.initialize(robot);

    robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);




      robot.vuforia.start();
      robot.vuforia.vuforiaMain(robot);
      imuUpdater.start();

      CameraDevice.getInstance().setFlashTorchMode(true);

    telemetry.addData("Robot Status:", "Initialized.");
    telemetry.update();


    waitForStart();

    //robot.autoStrafeDistanceSensor(turnPID, 1, 90, 0, 16);
      robot.autoStrafeDistanceSensorLeftorRight(turnPID, 0.75, "right", 16);
      robot.turnRobotAutonomous(0, 0, turnPID);

      sleep(2000);
      switch(robot.getBlockSwitchCase(robot.vuforia.blockXValue, 1)){
          case 1:
                robot.autoStrafeTime(turnPID, 1, 0,0,3);
              break;
          case 2:
              robot.autoStrafeTime(turnPID, 1, -90,0,3);

              break;


          case 3:
              robot.autoStrafeTime(turnPID, 1, 180,0,3);


              break;
      }

      CameraDevice.getInstance().setFlashTorchMode(false);
      //robot.autoStrafeDistanceSensor(turnPID, -1, -90, 0, 0);
      //robot.autoStrafe(turnPID, 1, 0, 180, 2);
      //robot.autoStrafe(turnPID, 1, -180, 0, 1);


      //robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, 0, 0);



//    while (robot.getWallDistanceInches() < 10) {
//      robot.autoStrafe(turnPID, -1, 0, 0);
//    }
    //robot.stopDriveMotors();
   // robot.turnRobotAutonomous(0, 500, turnPID);
    //robot.stopDriveMotors();
//




//    robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, 0, 100);
//
//      robot.turnRobotAutonomous(-90, 500, turnPID);
//
//    robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, -90, 60);
//
//    robot.turnRobotAutonomous(-180, 500, turnPID);
//
//    robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, -180, 80);
//
//    robot.turnRobotAutonomous(-270, 500, turnPID);
//
//
//    robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, -270, 60);
//
//    robot.turnRobotAutonomous(-360, 500, turnPID);
//
//    robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, -360, -19);



    while(true&&opModeIsActive()){
        robot.tempDouble1 = robot.vuforia.blockXValue;

    }


    // REMEMBER TO SHUT DOWN ALL THREADS
    imuUpdater.interrupt();
  }




  private class integrationOfAxis extends Thread
  {
    public integrationOfAxis()
    {
      this.setName("DriveThread");
    }


    @Override
    public void run()
    {
      while (!isInterrupted())
      {
          robot.vuforia.vuforiaMain(robot);

        robot.getIntegratedZAxis();
        robot.callAllTelemetry(); // Do not call telem.update at ALL. It'll freak out.

        idle();
      }
    }
  }


}
