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


@Autonomous(name = "Test", group = "ready")
//@Disabled
public class testProgram extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotTemplate robot;
    private double time;
    NickPID turnPID; // For the love of god don't forget to feed in robot object





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

        double robotDriveToBlock = robot.getBlockDistanceInches() -13;
        double robotDriveToWall = robot.getWallDistanceInches() - 3.3;

        waitForStart();
        runtime.reset();
        time = runtime.milliseconds();

        robot.autonomousNewMechDriveGradual(turnPID, 1, 0, 0, 15, 0.3, 0, 25, true); // 0.3 and 0.3 for short distances\

        // robot.autoMechanumSlideTime(turnPID, false,0.4,-90, 0,1);


        //robot.newBlockCase(turnPID, robot.blockPosition(turnPID, 5));

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