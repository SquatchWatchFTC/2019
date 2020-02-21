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

        servoMove servoTest = new servoMove(10, false);
        robot.resetEncoderWheels();

        imuUpdater.start();

        robot.initVoltage = robot.getBatteryVoltage();


        telemetry.addData("Robot Status:", "Initialized.");
        telemetry.update();

        //robot.readyToGrab();

        robot.rightReadyToGrab();
        waitForStart();

        double power = (((-0.2*1.01)*robot.initVoltage)+3.4);
        double deriv = 0;



        robot.rightReadyToGrabExtend();
        sleep(5000);
        robot.rightReadyToDrop();
        sleep(500);



        /*
        robot.turnRobotAutonomous(90, 0, turnPID, power,deriv,2);

        robot.turnRobotAutonomous(180,0, turnPID, power, deriv,2);

        robot.turnRobotAutonomous(90, 0, turnPID, power,deriv,2);

        robot.turnRobotAutonomous(0,0, turnPID, power, deriv,2);

        robot.turnRobotAutonomous(-90, 0, turnPID, power,deriv,2);

        robot.turnRobotAutonomous(-180, 0, turnPID, power,deriv,2);

        robot.turnRobotAutonomous(0, 0, turnPID, power+0.5,deriv,2);



        robot.turnRobotAutonomous(10, 0, turnPID, power/6,deriv,3);

        robot.turnRobotAutonomous(0,0, turnPID, power/6, deriv,2);

*/

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
                    //robot.automatedPickUpTele(thing, 350);
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

                   // robot.automatedCarriageReturnTele(thing, 0);
                    thing = false;
                }
                idle();
            }
        }
    }

    private class servoMove extends Thread
    {
        double distance;
        boolean grab;
        public servoMove(double distanceTilActivation, boolean grabBlock)
        {
            this.setName("servoThing");
            distance = distanceTilActivation;
            grab = grabBlock;
        }


        @Override
        public void run()
        {
            boolean thing = false;

            while (!isInterrupted() && !thing)
            {
                if(Math.abs(((((8*Math.PI)/2400)*robot.leftFront.getCurrentPosition()/2) + ((8*Math.PI)/2400)*robot.rightFront.getCurrentPosition()/2)/2) > Math.abs(distance)){
                    if(grab){
                        robot.leftDrop();
                    }else{
                        robot.grabLeftArm();
                    }
                    thing = true;
                }
                idle();
            }
        }
    }



}
