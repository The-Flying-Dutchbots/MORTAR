package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClimberSys;

@TeleOp
public class ClimberResetMode extends OpMode {
ClimberSys climberSys = new ClimberSys();
    @Override
    public void init() {
        climberSys.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            climberSys.manualMoveClimbersUpOveride();
        } else if (gamepad1.dpad_down) {
            climberSys.manualMoveClimbersDownOveride();
        }else{
            climberSys.stopClimbers();
        }
    }
}
