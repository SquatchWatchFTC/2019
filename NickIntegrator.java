package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class NickIntegrator{
	double currentValue=0;
	double previousValue=0;
	RobotTemplate robot;
	NickIntegrator(RobotTemplate robotObject){
		robot = robotObject;
	}
	public void integrator(double value, boolean telemetryOption){
		double deltaValue = value - this.previousValue;

		currentValue += deltaValue;

		this.previousValue = this.currentValue;

		if(telemetryOption){
			robot.autoOpMethods.telemetry.addData("integratedZAxis: ", robot.integratedZAxis);
			robot.autoOpMethods.telemetry.update();
		}
	}
}
