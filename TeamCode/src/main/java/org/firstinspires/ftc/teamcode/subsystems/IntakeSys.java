package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class IntakeSys {

    CRServo indexerServo;
    CRServo indexerServoTwo;
    DcMotor intakeMotor;

    DigitalChannel touchOne,touchTwo,beamBreak;
    private ElapsedTime touchTimer = new ElapsedTime();

    public void init(HardwareMap hwmap){
        indexerServoTwo = hwmap.get(CRServo.class, "indexer_servo_two");
        indexerServo = hwmap.get(CRServo.class, "indexer_servo");
        indexerServo.setDirection(CRServo.Direction.REVERSE);
        indexerServoTwo.setDirection(CRServo.Direction.FORWARD);

        intakeMotor = hwmap.get(DcMotor.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);


       // beamBreak = hwmap.get(DigitalChannel.class, "beam_break");
       // beamBreak.setMode(DigitalChannel.Mode.INPUT);

        touchOne = hwmap.get(DigitalChannel.class, "touch_one");
        touchTwo = hwmap.get(DigitalChannel.class, "touch_two");
        touchOne.setMode(DigitalChannel.Mode.INPUT);
        touchTwo.setMode(DigitalChannel.Mode.INPUT);

        beamBreak = hwmap.get(DigitalChannel.class, "beam_break");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
    }
public void intakeStart(){
        boolean isWaiting = ((!touchOne.getState()) || (!touchTwo.getState()));
        boolean beamBreak_broken = !beamBreak.getState();

    intakeMotor.setPower(Constants.INTAKE_POWER);
   if(beamBreak_broken){
       indexerServo.setPower(0);
       indexerServoTwo.setPower(0);
   } else if (isWaiting) {
        indexerServo.setPower(Constants.INDEXER_INTAKE_POWER);
       indexerServoTwo.setPower(Constants.INDEXER_INTAKE_POWER);
        touchTimer.reset();
    } else if(touchTimer.seconds() <= Constants.TOUCH_SENSOR_TIMER) {
        indexerServo.setPower(Constants.INDEXER_INTAKE_POWER);
       indexerServoTwo.setPower(Constants.INDEXER_INTAKE_POWER);
    } else{
       indexerServo.setPower(0);
       indexerServoTwo.setPower(0);
    }

}
public boolean getTouchOne(){
       return touchOne.getState();
}
    public boolean getTouchTwo(){
        return touchTwo.getState();
    }


    public boolean getBeamBreak() {
        return beamBreak.getState();
    }

    public void indexerStart(){
        indexerServo.setPower(Constants.INDEXER_INTAKE_POWER);
        indexerServoTwo.setPower(Constants.INDEXER_INTAKE_POWER);
}
public void indexerStop(){
        indexerServo.setPower(0);
        indexerServoTwo.setPower(0);
}
public void indexerReverse(){
        indexerServo.setPower(-Constants.INDEXER_INTAKE_POWER);
        indexerServoTwo.setPower(-Constants.INDEXER_INTAKE_POWER);
}
public void shootStart(){
        intakeMotor.setPower(Constants.INTAKE_POWER);
        indexerServo.setPower(Constants.INDEXER_SHOOT_POWER);
        indexerServoTwo.setPower(Constants.INDEXER_SHOOT_POWER);
    }
    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }
public void intakeStop(){
        intakeMotor.setPower(0);
        indexerServo.setPower(0);
        indexerServoTwo.setPower(0);
}
public void intakeReverse(){
        intakeMotor.setPower(-0.3);
        indexerServo.setPower(-0.8);
        indexerServoTwo.setPower(-0.8);
}


}
