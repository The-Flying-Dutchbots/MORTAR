package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSys;

@Autonomous
public class FarRedAuto extends OpMode {


    ElapsedTime ShootTimer = new ElapsedTime();
    ElapsedTime UtilityTimer = new ElapsedTime();
    IntakeSys intakeSys = new IntakeSys();
    ShooterSys shooterSys = new ShooterSys();
    ArcadeDriveSys driveSys = new ArcadeDriveSys();
    private CloseRedAuto.AutoState autoState = CloseRedAuto.AutoState.REV_SHOOTER_1;

    @Override
    public void init() {
        intakeSys.init(hardwareMap);
        shooterSys.init(hardwareMap);
        driveSys.autoInit(hardwareMap);


    }

    public void loop() {
        switch (autoState) {

            case REV_SHOOTER_1:
                shooterSys.setHoodState(ShooterSys.HoodState.FAR);
                shooterSys.startShooting();
                UtilityTimer.reset();

                autoState = CloseRedAuto.AutoState.REV_SHOOTER_1_WAIT;


                break;
            case REV_SHOOTER_1_WAIT:

                if(UtilityTimer.seconds() > 0.5) {
                    autoState = CloseRedAuto.AutoState.SHOOT_1;
                    UtilityTimer.reset();
                }
                break;
            case SHOOT_1:
                if(UtilityTimer.seconds() > 1.0) {
                    intakeSys.shootStart();
                    UtilityTimer.reset();
                    shooterSys.startShooting();
                    autoState = CloseRedAuto.AutoState.SHOOTING_WAIT_1;
                }
                break;

            case SHOOTING_WAIT_1:
                if (UtilityTimer.seconds() > 5.0) {
                    shooterSys.stopShooting();
                    intakeSys.intakeStop();
                    autoState = CloseRedAuto.AutoState.MOVE_FORWARD_ONE;
                }
                break;

            // --- Move forward 20 inches ---
            case MOVE_FORWARD_ONE:
                driveSys.moveFowardInchesAtPower(-18, 0.5);  // sets RUN_TO_POSITION
                autoState = CloseRedAuto.AutoState.MOVE_FORWARD_ONE_WAIT;
                break;

            // --- Wait until motors finish moving ---
            case MOVE_FORWARD_ONE_WAIT:
                if (!driveSys.isBusy()) {
                    UtilityTimer.reset();   // start 0.5s delay
                    autoState = CloseRedAuto.AutoState.MOVE_FORWARD_ONE_DELAY;
                }
                break;

            // --- Wait 0.5s after reaching target ---
            case MOVE_FORWARD_ONE_DELAY:

                if (UtilityTimer.seconds() > 0.5) {
                    autoState = CloseRedAuto.AutoState.MOVE_ROTATE_ONE;  // next action
                }
                break;
            case MOVE_ROTATE_ONE:
                intakeSys.intakeStart();
                driveSys.turnDegrees(110, 0.5);
                UtilityTimer.reset();
                autoState = CloseRedAuto.AutoState.MOVE_ROTATE_ONE_WAIT;

                break;
            case MOVE_ROTATE_ONE_WAIT:

                if (!driveSys.isBusy() || UtilityTimer.seconds() > 2) {

                    UtilityTimer.reset();
                    autoState = CloseRedAuto.AutoState.MOVE_ROTATE_ONE_DELAY;

                }
                break;
            case MOVE_ROTATE_ONE_DELAY:

                if (UtilityTimer.seconds() > 0.5) {

                    autoState = CloseRedAuto.AutoState.MOVE_FORWARD_INTAKE;
                }
                break;
            case MOVE_FORWARD_INTAKE:


                driveSys.moveFowardInchesAtPower(20, 0.2);
                UtilityTimer.reset();


                autoState = CloseRedAuto.AutoState.MOVE_FORWARD_INTAKE_WAIT;

                break;
            case MOVE_FORWARD_INTAKE_WAIT:
                intakeSys.intakeStart();
                if (UtilityTimer.seconds() > 3.3) {

                    UtilityTimer.reset();
                    intakeSys.intakeStart();
                    autoState = CloseRedAuto.AutoState.MOVE_BACKWARD_INTAKE;
                }
                break;

            case MOVE_BACKWARD_INTAKE:
                intakeSys.intakeStart();
                driveSys.moveFowardInchesAtPower(-20, 0.3);
                autoState = CloseRedAuto.AutoState.MOVE_BACKWARD_INTAKE_WAIT;
                break;
            case MOVE_BACKWARD_INTAKE_WAIT:
                intakeSys.intakeStart();
                if (!driveSys.isBusy()) {
                    intakeSys.intakeStart();
                    UtilityTimer.reset();
                    autoState = CloseRedAuto.AutoState.MOVE_ROTATE_TW0;
                }
                break;

            case MOVE_ROTATE_TW0:
                intakeSys.intakeStop();
                driveSys.turnDegrees(-110, 0.5);
                autoState = CloseRedAuto.AutoState.MOVE_ROTATE_TWO_WAIT;
                break;
            case MOVE_ROTATE_TWO_WAIT:
                if (!driveSys.isBusy()) {
                    UtilityTimer.reset();
                    autoState = CloseRedAuto.AutoState.MOVE_ROTATE_TWO_DELAY;
                }
                break;
            case MOVE_ROTATE_TWO_DELAY:
                if (UtilityTimer.seconds() > 0.5) {
                    autoState = CloseRedAuto.AutoState.MOVE_FORWARD_TWO;
                }
                break;
            case MOVE_FORWARD_TWO:
                driveSys.moveFowardInchesAtPower(16.5, 0.5);
                autoState = CloseRedAuto.AutoState.MOVE_FORWARD_TWO_WAIT;
                break;
            case MOVE_FORWARD_TWO_WAIT:
                if(!driveSys.isBusy()){
                    autoState = CloseRedAuto.AutoState.REV_SHOOTER_2;
                }
            case REV_SHOOTER_2:
                shooterSys.startShooting();
                ShootTimer.reset();
                autoState = CloseRedAuto.AutoState.SHOOT_2;
                break;
            case SHOOT_2:
                if (ShootTimer.seconds() > 1.0) {
                    UtilityTimer.reset();
                    intakeSys.shootStart();
                    autoState = CloseRedAuto.AutoState.SHOOTING_WAIT_2;
                }
                break;
            case SHOOTING_WAIT_2:
                if (UtilityTimer.seconds() > 5.5) {
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    shooterSys.setHoodState(ShooterSys.HoodState.STOWED);
                    autoState = CloseRedAuto.AutoState.MOVE_ROTATE_THREE;
                }
                break;
            case MOVE_ROTATE_THREE:
                driveSys.turnDegrees(0, 0.8);
                autoState = CloseRedAuto.AutoState.MOVE_ROTATE_THREE_WAIT;
                UtilityTimer.reset();
                break;
            case MOVE_ROTATE_THREE_WAIT:
                if (UtilityTimer.seconds() > 0.1) {
                    autoState = CloseRedAuto.AutoState.MOVE_FINAL;

                }
                break;
            case MOVE_FINAL:
                driveSys.moveFowardInchesAtPower(-18, 1.0);
                autoState = CloseRedAuto.AutoState.MOVE_FINAL_WAIT;
                break;


        }

        telemetry.addData("Auto State", autoState);
        shooterSys.updateHood();

    }
}