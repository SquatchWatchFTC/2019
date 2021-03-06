package org.firstinspires.ftc.teamcode;

public class NickPID{
    private double P, I, D = 0;
    private double kP, kI, kD = 0;
    public double previousError = 0;
    public double previousValue = 0;
    public double currentError = 0;
    double deltaError = 0;
    double staticCurrentValue = 0;
    boolean loopFlag = false;

    RobotTemplate robot;

    private boolean calcFlag1 = true;

    public NickPID(){
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
        this.currentError = 0;
        robot.errorTemp = 100;
        this.deltaError = 0;

        staticCurrentValue = 0;
        loopFlag = false;

        calcFlag1 = true;
    }

    public double genericPID(double targetValue, double currentValue, double kP, double kI, double kD, double bias){
        if(!loopFlag){
            staticCurrentValue = currentValue;
            loopFlag = true;
        }
        currentError = (targetValue - currentValue)/Math.abs(targetValue-staticCurrentValue);
        robot.errorTemp= currentError;

        P = currentError;

        I += currentError/100000;

        D = (currentError - previousError)/100;

        if(I > 0.30){
            I=0.3;
        }else if(I < -0.3){
            I=-0.3;
        }





        double totalError = (P * kP) + (I * kI) + (D * kD);

        if(totalError > 0){
            totalError += bias;
        }else{
            totalError -= bias;
        }

        if(totalError > 1){
            totalError = 1;
        }else if(totalError < -1){
            totalError = -1;
        }

        deltaError = currentValue - previousValue;
        previousError = currentError;
        previousValue = currentValue;
        return totalError;
    }
    public double basicPIDReturnShush(double targetAngle, double kP, double kI, double kD, boolean telemetryOption){
        double error = targetAngle + robot.integratedZAxis;
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
