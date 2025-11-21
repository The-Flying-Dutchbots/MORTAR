package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.objdetect.HOGDescriptor;

public class ShooterSys {

    public enum HoodState{
        STOWED,
        MID,
        FAR;

    }
    boolean isMoving;

    DcMotor shooterMotor;
    //DcMotorEx shooterMtr;
    CRServo hoodServo;



    private HoodState currentHoodState = HoodState.STOWED;
    private HoodState targetHoodState = HoodState.STOWED; // desired hood position

    private ElapsedTime stateTimer = new ElapsedTime();

    public void init(HardwareMap hwmap){

        isMoving = false;
        currentHoodState = HoodState.STOWED;
        targetHoodState  = HoodState.STOWED;

        stateTimer.reset();
        shooterMotor = hwmap.get(DcMotor.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

//shooter conversion to rpm controller begins here
     /*   shooterMtr = hwmap.get(DcMotorEx.class, "INSERT NAME?");
        shooterMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMtr.setVelocityPIDFCoefficients(0,0,0,0);*/

        hoodServo = hwmap.get(CRServo.class, "hood_servo");
        hoodServo.setDirection(CRServo.Direction.FORWARD);



    }

    public void setShooterPower(double shooterPower){
        shooterMotor.setPower(shooterPower);


    }
    public void startShooting(){
        if(currentHoodState == HoodState.STOWED){
            setShooterPower(Constants.CLOSE_SHOT_POWER);

        }
        else if(currentHoodState == HoodState.MID){
            setShooterPower(Constants.MID_SHOT_POWER);
        }
        else if(currentHoodState == HoodState.FAR){
            setShooterPower(Constants.FAR_SHOT_POWER);
        }


    }

    public void stopShooting(){
        setShooterPower(0);
    }
//begining of the hood control
    public void setHoodState(HoodState newState) {
        if (!isMoving && newState != currentHoodState) {
            targetHoodState = newState;
            isMoving = true;
            stateTimer.reset();
        }
    }

    public void updateHood() {


        double power = 0;
        double moveTime = 0;

        // Determine direction and duration based on current vs target
        if (currentHoodState == HoodState.STOWED) {
            if (targetHoodState == HoodState.MID) {
                power = Constants.HOOD_UP_POWER;
                moveTime = Constants.MOVE_TIME_STOWED_TO_MID;
            } else if (targetHoodState == HoodState.FAR) {
                power = Constants.HOOD_UP_POWER;
                moveTime = Constants.MOVE_TIME_STOWED_TO_FAR;
            }
        } else if (currentHoodState == HoodState.MID) {
            if (targetHoodState == HoodState.STOWED) {
                power = Constants.HOOD_DOWN_POWER;
                moveTime = Constants.MOVE_TIME_STOWED_TO_MID;
            } else if (targetHoodState == HoodState.FAR) {
                power = Constants.HOOD_UP_POWER;
                moveTime = Constants.MOVE_TIME_MID_TO_FAR;
            }
        } else if (currentHoodState == HoodState.FAR) {
            if (targetHoodState == HoodState.MID) {
                power = Constants.HOOD_DOWN_POWER;
                moveTime = Constants.MOVE_TIME_MID_TO_FAR;
            } else if (targetHoodState == HoodState.STOWED) {
                power = Constants.HOOD_DOWN_POWER;
                moveTime = Constants.MOVE_TIME_STOWED_TO_FAR;
            }
        }

        // Run the hood for the calculated time
        if (stateTimer.seconds() < moveTime) {
            hoodServo.setPower(power);
        } else {
            hoodServo.setPower(0);
            isMoving = false;
            currentHoodState = targetHoodState; // update current position
        }
    }

    public HoodState getCurrentHoodPos() { return currentHoodState; }
    public boolean getIsMoving() { return isMoving; }

    public HoodState getCurrentHoodState(){
        return  currentHoodState;
    }


}
