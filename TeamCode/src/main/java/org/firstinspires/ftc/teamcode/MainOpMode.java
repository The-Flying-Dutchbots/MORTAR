package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;
import org.firstinspires.ftc.teamcode.subsystems.ClimberSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystems.LightSys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSys;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

@TeleOp
public class MainOpMode extends OpMode {
    VisionSys visionSys = new VisionSys();
    ArcadeDriveSys driveSys = new ArcadeDriveSys();
    ShooterSys shooterSys = new ShooterSys();
    IntakeSys intakeSys = new IntakeSys();
    ClimberSys climberSys = new ClimberSys();
    LightSys lightSys = new LightSys();
    ElapsedTime intakeShootTimer = new ElapsedTime();

    boolean isShooting = false;

    double foward, rotate, strafe;

    @Override
    public void init() {
        driveSys.init(hardwareMap);
        shooterSys.init(hardwareMap);
        intakeSys.init(hardwareMap);
        climberSys.init(hardwareMap);
        visionSys.init(hardwareMap);
        lightSys.init(hardwareMap);


    }

    public void loop(){

        telemetry.addData("limelight ID seen:", visionSys.getTagID());
    //Drive chassis controls Telemetry and data



        double robotAngle = driveSys.getYaw();


        telemetry.addData("robotYaw",driveSys.getYaw());
        foward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;

        /*if(gamepad1.x) {// Face forward
            if(shooterSys.getCurrentHoodPos() == ShooterSys.HoodState.FAR){
                rotate = driveSys.getTurnPowerToHeading(112);
            }
            else {
                rotate = driveSys.getTurnPowerToHeading(135);
            }

        } else if(gamepad1.b) {    // Face right
            if(shooterSys.getCurrentHoodPos() == ShooterSys.HoodState.FAR){
                rotate = driveSys.getTurnPowerToHeading(-114);
            }
            else {
                rotate = driveSys.getTurnPowerToHeading(-135);
            }
        } */
        if(gamepad1.x){
            visionSys.setPipeline(Constants.BLUE_AIMING_PIPELINE);
            LLResult result = visionSys.getLatestResult();
            if(result != null && result.isValid()){
                rotate = driveSys.limeLightAim(visionSys.getTx());
            }else if(shooterSys.getCurrentHoodPos() == ShooterSys.HoodState.FAR){
                rotate = driveSys.getTurnPowerToHeading(112);
            }
            else {
                rotate = driveSys.getTurnPowerToHeading(135);
            }
        } else if(gamepad1.b){
            visionSys.setPipeline(Constants.RED_AIMING_PIPELINE);
            LLResult result = visionSys.getLatestResult();
            if(result != null && result.isValid()){
                rotate = driveSys.limeLightAim(visionSys.getTx());
            }else if(shooterSys.getCurrentHoodPos() == ShooterSys.HoodState.FAR){
                rotate = driveSys.getTurnPowerToHeading(-114);
            }
            else {
                rotate = driveSys.getTurnPowerToHeading(-135);
            }
        }else {
            // Normal manual turning
            rotate = -gamepad1.right_stick_x;
        }



        driveSys.FieldOrientedDrive(foward, strafe, rotate);





    // Driver controls for comp setup
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            driveSys.ResetPose();
        }


        if((gamepad2.right_trigger > 0.5) && !isShooting){
            shooterSys.startShooting();
            intakeShootTimer.reset();
            isShooting = true;
        }else if((gamepad2.right_trigger > 0.5) && (intakeShootTimer.seconds() > 1.5) && shooterSys.getCurrentHoodState() == ShooterSys.HoodState.FAR){
            intakeSys.shootStart();
        }
        else if((gamepad2.right_trigger > 0.5) && (intakeShootTimer.seconds() > Constants.SHOOT_WAIT_TIMER) && shooterSys.getCurrentHoodState() != ShooterSys.HoodState.FAR){
            intakeSys.shootStart();
        }
        else if(gamepad2.right_trigger > 0.5){
            shooterSys.startShooting();
        }
        else if ((gamepad1.right_trigger > 0.5) && gamepad1.right_bumper){
           intakeSys.intakeStart();
           intakeSys.indexerStart();
           shooterSys.stopShooting();
           isShooting = false;
        }else if ((gamepad1.right_trigger > 0.5) && gamepad1.left_bumper){
            intakeSys.intakeStart();
            intakeSys.indexerReverse();
            shooterSys.stopShooting();
            isShooting = false;
        } else if (gamepad1.right_trigger > 0.5){
            intakeSys.intakeStart();
            shooterSys.stopShooting();
            isShooting = false;
        }else if(gamepad1.left_trigger > 0.5){
            intakeSys.intakeReverse();
            shooterSys.setShooterRPM(-0.5);

        }else{
          intakeSys.intakeStop();
          shooterSys.stopShooting();

          isShooting = false;
        }

        if(gamepad1.right_bumper){
            intakeSys.indexerStart();
        }

        if(gamepad1.left_bumper){
            intakeSys.indexerReverse();
        }


        if (gamepad2.a) {
            shooterSys.setHoodState(ShooterSys.HoodState.STOWED);
        }
        else if (gamepad2.x) {
            shooterSys.setHoodState(ShooterSys.HoodState.MID);
        }
        else if (gamepad2.y) {
            shooterSys.setHoodState(ShooterSys.HoodState.FAR);
        }

       if(gamepad2.dpad_up){
            climberSys.manualMoveClimbersUp();
        }
        else if(gamepad2.dpad_down){
            climberSys.manualMoveClimbersDown();
        }
        else {
            climberSys.stopClimbers();
       }

        if(gamepad2.right_trigger > 0.5){
            lightSys.setLightState(LightSys.LightState.shooting);
        } else if (gamepad1.right_trigger > 0.5 && !intakeSys.getBeamBreak()) {
            lightSys.setLightState(LightSys.LightState.beamBroken);
        } else if (gamepad1.right_trigger > 0.5 && shooterSys.getCurrentHoodState() == ShooterSys.HoodState.FAR)  {
            lightSys.setLightState(LightSys.LightState.far_intaking);
        } else if (gamepad1.right_trigger > 0.5 && shooterSys.getCurrentHoodState() == ShooterSys.HoodState.MID)  {
            lightSys.setLightState(LightSys.LightState.mid_intaking);
        } else if (gamepad1.right_trigger > 0.5 && shooterSys.getCurrentHoodState() == ShooterSys.HoodState.STOWED)  {
            lightSys.setLightState(LightSys.LightState.close_intaking);
        } else if (shooterSys.getCurrentHoodState() == ShooterSys.HoodState.FAR) {
            lightSys.setLightState(LightSys.LightState.far);
        } else if (shooterSys.getCurrentHoodState() == ShooterSys.HoodState.MID){
            lightSys.setLightState(LightSys.LightState.mid);
        }else if (shooterSys.getCurrentHoodState() == ShooterSys.HoodState.STOWED){
            lightSys.setLightState(LightSys.LightState.close);
        }

        // Shooter Sys Telemetry and update control. do not adjust order, updateHood must remain post controls and
        // pre telemetry.update();
        shooterSys.updateHood();

        telemetry.addData("Hood Pos", shooterSys.getCurrentHoodPos());
        telemetry.addData("Is Moving", shooterSys.getIsMoving());
        telemetry.addData("climber_pos", climberSys.getClimberPositionAvg());

        telemetry.addData("Touch One",intakeSys.getTouchOne());
        telemetry.addData("Touch Two",intakeSys.getTouchTwo());
        telemetry.addData("Beam Break", intakeSys.getBeamBreak());

        telemetry.addData("goal rpm",shooterSys.getTargetRPM());
        telemetry.addData("current rpm", shooterSys.getShooterRPM());
        telemetry.addData("pinpoint heading", driveSys.getPinpointYaw());
        telemetry.update();
        lightSys.updateLights();




    }
}

