package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Red Side TeleOp", group="Exercises")
//@Disabled
public class RedSideTele extends LinearOpMode
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
                robot.robotDriveFunctions(turnPID, gamepad1.x, true);
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


