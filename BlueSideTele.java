package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="Blue Side TeleOp", group="Exercises")
//@Disabled
public class BlueSideTele extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private RobotTemplate robot;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // create an instance of the DriveThread.
        robot = new RobotTemplate(this);
        robot.autoInit();
        robot.resetEncoderWheels();
        
        Thread  intakeAndLift = new intakeAndLift();
        NickPID turnPID = new NickPID(robot);

        //robot.resetEncoderWheels();

        telemetry.addData("Mode: ", "Initialized."); // Once you see this, the calibration and vuforia is initialized.
        telemetry.update();

        // wait for start button.



        waitForStart();        // wait for start button.


        intakeAndLift.start();
        robot.liftHeightColor.start();
     //   robot.teleOpAutoMatedThread.start();





            while (opModeIsActive())
            {
                robot.getIntegratedZAxis();
                robot.autoOpMethods.telemetry.addData("integratedZAxis: ", robot.integratedZAxis);
                robot.leftRear.getPower();

//                robot.autoOpMethods.telemetry.addData("leftEnc: ", robot.leftFront.getCurrentPosition());
//                robot.autoOpMethods.telemetry.addData("rightEnc: ", robot.rightFront.getCurrentPosition());
                robot.autoOpMethods.telemetry.update();
                robot.robotDriveFunctions(turnPID, gamepad1.x, false);
                idle();
            }

        // stop the driving thread.
        intakeAndLift.interrupt();
        robot.liftHeightColor.interrupt();
     //   robot.teleOpAutoMatedThread.interrupt();


    }

    private class intakeAndLift extends Thread
    {
        public intakeAndLift()
        {
            this.setName("DriveThread");
            robot.initialLiftEncoderCount = robot.liftMotor.getCurrentPosition();

        }

        // called when thread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {

                while (!isInterrupted())
                {

                    robot.primaryProgram();


                    idle();

                }
      



        }
    }

}


