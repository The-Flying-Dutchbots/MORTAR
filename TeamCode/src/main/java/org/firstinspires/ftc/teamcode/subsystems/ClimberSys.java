package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ClimberSys {

    DcMotor leftClimberMotor, rightClimberMotor;


    public void init(HardwareMap hwmap){
        leftClimberMotor = hwmap.get(DcMotor.class, "left_climber_motor");
        rightClimberMotor = hwmap.get(DcMotor.class, "right_climber_motor");

        leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftClimberMotor.setDirection(DcMotor.Direction.REVERSE);
        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimberMotor.setDirection(DcMotor.Direction.FORWARD);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightClimberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setClimberPosition(int position){
        leftClimberMotor.setTargetPosition(position);
        leftClimberMotor.setTargetPosition(position);
    }

    public void manualMoveClimbersUp(){
if(getClimberPositionAvg() < 15800) {
    leftClimberMotor.setPower(Constants.CLIMBER_MANUAL_POWER);
    rightClimberMotor.setPower(Constants.CLIMBER_MANUAL_POWER);
}else{
    leftClimberMotor.setPower(0);
    rightClimberMotor.setPower(0);
}
    }
    public void manualMoveClimbersDown(){
if(getClimberPositionAvg() > 100) {
    leftClimberMotor.setPower(-Constants.CLIMBER_MANUAL_POWER);
    rightClimberMotor.setPower(-Constants.CLIMBER_MANUAL_POWER);
}
else{
    leftClimberMotor.setPower(0);
    rightClimberMotor.setPower(0);
}
    }
    public void stopClimbers(){

        leftClimberMotor.setPower(0);
        rightClimberMotor.setPower(0);
    }
    public void manualMoveClimbersUpOveride(){
            leftClimberMotor.setPower(Constants.CLIMBER_MANUAL_POWER);
            rightClimberMotor.setPower(Constants.CLIMBER_MANUAL_POWER);
    }
    public void manualMoveClimbersDownOveride(){
            leftClimberMotor.setPower(-Constants.CLIMBER_MANUAL_POWER);
            rightClimberMotor.setPower(-Constants.CLIMBER_MANUAL_POWER);
    }
    public int getLeftClimberPosition(){
        return leftClimberMotor.getCurrentPosition();
    }

    public int getRightClimberPosition(){
        return rightClimberMotor.getCurrentPosition();
    }

    public int getClimberPositionAvg(){
        return (leftClimberMotor.getCurrentPosition() + rightClimberMotor.getCurrentPosition())/2;
    }
}
