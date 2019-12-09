package org.firstinspires.ftc.teamcode;

public class NickPID{
	private double P, I, D = 0;
	private double kP, kI, kD = 0;
	public double previousError = 0;
	RobotTemplate robot;

	private boolean calcFlag1 = true;

	public NickPID(){ 
		//Why include this?
	}

	public NickPID(RobotTemplate robotObject){
		robot = robotObject;
	}
	public void resetValues(){
		this.P = 0;
		this.I = 0;
		this.D = 0;

		this.kP = 0;
		this.kI = 0;
		this.kD = 0;

		this.previousError = 0;
		robot.errorTemp = 100;

		calcFlag1 = true;
	}

	public double basicPIDReturnShush(double targetAngle, double kP, double kI, double kD, boolean telemetryOption){
		double error = targetAngle + robot.integratedZAxis;
		error = -error;
		robot.errorTemp = error;
		this.P = error;
		this.I += (error*.02);
		this.D = (error - this.previousError);

		double output = (kP * this.P) + (kI * this.I) + (kD*this.D);

		if(telemetryOption){
			robot.autoOpMethods.telemetry.addData(this.getClass().getName() + "PID Error: ", error);
		}

		if(I > 50){
			I=50;
		}

		if(output > 100){
			output = 100;
		}

		this.previousError = error;
		return output;
	}
	public double basicPIDReturn(double targetAngle, double kP, double kI, double kD, boolean telemetryOption){
		double error = -targetAngle - robot.integratedZAxis;
		robot.errorTemp = error;
		this.P = error;
		this.I += (error*.02);
		this.D = (error - this.previousError);

		double output = (kP * this.P) + (kI * this.I) + (kD*this.D);

		if(telemetryOption){
			robot.autoOpMethods.telemetry.addData(this.getClass().getName() + "PID Error: ", error);
		}

		if(I > 50){
			I=50;
		}

		if(output > 100){
			output = 100;
		}

		this.previousError = error;
		return output;
	}

	public double basicPIDReturnGeneral(double targetValue, int currentValue,  double kP, double kI, double kD, boolean telemetryOption){
		double error = targetValue - currentValue;
		robot.errorTemp = error;
		this.P = error;
		this.I += (error*.02);
		this.D = (error - this.previousError);

		double output = (kP * this.P) + (kI * this.I) + (kD*this.D);

		if(telemetryOption){
			robot.autoOpMethods.telemetry.addData(this.getClass().getName() + "PID Error: ", error);
		}

		if(I > 50){
			I=50;
		}

		if(output > 100){
			output = 100;
		}

		this.previousError = error;
		return output;
	}
	private double autoPIDReturn(double targetAngle, double pU, double tU, boolean telemetryOption){ // Time unit, which is period of oscillation found in basicPIDReturn, and Pu is what you found to be causing the oscillation.
		if(this.calcFlag1){
			this.kP = 0.6 * pU;
			this.kI = (1.2 * pU)/tU;
			this.kD = (3 * pU * tU)/40;
		}else{
			this.calcFlag1 = false;
		}
		double error = -targetAngle - robot.integratedZAxis;

		this.P = error;
		this.I += (error*.02);
		this.D = (error - this.previousError)/0.02;

		double output = (this.kP * this.P) + (this.kI * this.I) + (this.kD*this.D);

		if(telemetryOption){
			robot.autoOpMethods.telemetry.addData(this.getClass().getName() + " AutoPID Error: ", error);
		}

		this.previousError = error;
		return output;
	}
}
