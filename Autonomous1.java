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
import org.firstinspires.ftc.robotcore.internal.tfod.Timer;


@Autonomous(name = "Autonomous 1.21", group = "test")
//@Disabled
public class Autonomous1 extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private RobotTemplate robot;

	private double timeout;

	@Override
		public void runOpMode() {

			//initialize the robot object
			robot = new RobotTemplate(this);
			//add initial telemetry
			telemetry.addData("Robot Status:", "Initializing, Please Wait.");
			telemetry.update();

			//attempt to initiailize the robot
			robot.autoInit();

			NickPID turnPID = new NickPID(robot); // For the love of god don't forget to feed in robot object
			robot.vuforia = new VuforiaSkystone1920();
			integrationOfAxis imuUpdater = new integrationOfAxis();
			liftThing doLiftPickup = new liftThing(true);
			liftThing doLiftReturn = new liftThing(false);

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
			//robot.autoStrafeDistanceSensorLeftorRight(turnPID, 0.75, "left", 16);

			robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, 0, 20);
			robot.turnRobotAutonomous(90, 0, turnPID);
			sleep(500);
			CameraDevice.getInstance().setFlashTorchMode(false);

			switch(robot.getBlockSwitchCase(robot.vuforia.blockXValue, 0.5)) {
				case 1: // Right Block
					robot.autoMechanumDriveEncoder(turnPID, false, -0.5, 0, 0, 4);
					robot.turnRobotAutonomous(5, 0, turnPID);
					robot.turnRobotAutonomous(0, 0, turnPID);

					robot.autoMechanumDriveEncoder(turnPID, false, 0.5, 0, 0, 24);

					//robot.autoStrafeDistanceSensorLeftorRight(turnPID, 0.75, "left", 38);
					robot.turnRobotAutonomous(-45, 0, turnPID);

					robot.turnRobotAutonomous(90, 0, turnPID);
					//
					robot.intake(true, false, 0.73);
					robot.autoMechanumDriveTime(turnPID, false, 0.5, 0,90, 1);
					robot.autoMechanumDriveEncoder(turnPID, false, -0.5, 0, 0, 4);
					sleep(800);
					robot.intake(false, false, 0.5);
					//
					robot.turnRobotAutonomous(45, 0, turnPID);
					robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 20);
					robot.turnRobotAutonomous(85, 0, turnPID);

					//robot.autoStrafeDistanceSensorLeftorRight(turnPID, -0.75, "right", 23);

					robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 28);
					doLiftPickup.start();
					robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 20);

					robot.turnRobotAutonomous(170, 0, turnPID);
					robot.backupToPlate(2, .5);
					//robot.autoMechanumDriveEncoder(turnPID, false, -0.5, 0, 0, 11);

					robot.leftDragServo.setPosition(.7);
					robot.rightDragServo.setPosition(.2);
					sleep((500));
					robot.gripServo.setPosition(.76);

					sleep(1000);

					doLiftReturn.start();
					/*
					   while (robot.autoOpMethods.opModeIsActive() && runtime.milliseconds() < 28500) {
					   robot.autoMechanumDriveEncoder(turnPID, false, 1, 0, 20, 28.25);
					   }
					 */

					robot.autoMechanumDriveTime(turnPID, false, 1, 0,180, 2);

					robot.leftDragServo.setPosition(0.1);
					robot.rightDragServo.setPosition(1);

					//robot.autoMechanumDriveTime(turnPID, false, 1, -90,180, 4);
					robot.autoMechanumDriveTime(turnPID, false, 1, -90,180, 1);
					robot.autoMechanumDriveTime(turnPID, false, 1, -85,90, 3);

					break;
				case 2: // Middle Block
					telemetry.addLine("case 2");
					robot.autoMechanumDriveEncoder(turnPID, false, .65, 0, 0, 11);

					robot.turnRobotAutonomous(5, 0, turnPID);
					robot.turnRobotAutonomous(0, 0, turnPID);

					robot.autoMechanumDriveEncoder(turnPID, false, 0.5, 0, 0, 24.5);

					robot.turnRobotAutonomous(-55, 0, turnPID);
					robot.turnRobotAutonomous(90, 0, turnPID);

					robot.intake(true, false, 0.73);
					robot.autoMechanumDriveTime(turnPID, false, 0.5, 0,90, 1);
					robot.autoMechanumDriveEncoder(turnPID, false, -0.5, 0, 0, 2.75);
					sleep(800);
					robot.intake(false, false, 0.5);

					robot.turnRobotAutonomous(45, 0, turnPID);robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 20);
					robot.turnRobotAutonomous(90, 0, turnPID);

					robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 44);
					doLiftPickup.start();
					robot.autoMechanumDriveEncoder(turnPID, false, -1, 0, 0, 20);

					robot.turnRobotAutonomous(180, 0, turnPID);
					robot.backupToPlate(2, .5);


					if (true) {
						robot.leftDragServo.setPosition(.7);
						robot.rightDragServo.setPosition(.2);

					} else {
						robot.leftDragServo.setPosition(0.1);
						robot.rightDragServo.setPosition(1);
					}

					sleep((500));
					robot.gripServo.setPosition(.76);

					sleep(1000);

					doLiftReturn.start();

					robot.autoMechanumDriveTime(turnPID, false, 1, 0,180, 1.0);
					double startTime = time;
					while((time <  startTime + 2) && (robot.integratedZAxis < -90)){
						robot.turnRobotPower(1);
					}
					robot.turnRobotPower(0);
					if(false) {
						robot.leftDragServo.setPosition(.7);
						robot.rightDragServo.setPosition(.2);

					} else {
						robot.leftDragServo.setPosition(0.1);
						robot.rightDragServo.setPosition(1);
					}
					robot.autoMechanumDriveTime(turnPID, false, -1, 0,90, 1.0);

					robot.autoMechanumDriveTime(turnPID, false, 1, 0,90, 2.0);

					break;


				case 3: // Left Block
					robot.turnRobotAutonomous(180, 0, turnPID);

					while(true && opModeIsActive()) {
						telemetry.addData("IntegratedZ: ", robot.integratedZAxis);
						telemetry.update();
					}
					sleep(30000);
					doLiftPickup.start();

					sleep(30000);
					robot.leftDragServo.setPosition(.7);
					robot.rightDragServo.setPosition(.15);

					robot.autoMechanumDriveEncoder(turnPID, false, 0.8, 0, 0, 20);
					robot.turnRobotAutonomous(-90, 0, turnPID);

					robot.leftDragServo.setPosition(0.1);
					robot.rightDragServo.setPosition(1);
					break;
			}

			doLiftPickup.interrupt();
			doLiftReturn.interrupt();
			imuUpdater.interrupt();

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

			// REMEMBER TO SHUT DOWN ALL THREADS
		}

	private class integrationOfAxis extends Thread {
		public integrationOfAxis() {
			this.setName("integratedZAxisThread");
		}

		@Override
			public void run() {
				while (!isInterrupted()) {
					robot.vuforia.vuforiaMain(robot);

					robot.getIntegratedZAxis();
					robot.callAllTelemetry(); // Do not call telem.update at ALL. It'll freak out.

					idle();
				}
			}
	}

	private class liftThing extends Thread {
		boolean pickupFlag = false;
		public liftThing(boolean pickup) {
			pickupFlag = pickup;
			this.setName("liftThingThread");
		}

		@Override
			public void run() {
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
