package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;

@TeleOp
public class TurnAndDriveTest extends OpMode {
ArcadeDriveSys arcadeDriveSys = new ArcadeDriveSys();
    boolean aPrev = false;
    boolean movPrev = false;

    @Override
    public void init() {
        arcadeDriveSys.init(hardwareMap);
    }

    @Override
    public void loop() {
        boolean aCurr = gamepad1.left_bumper; // current A button state
        boolean MovCurr = gamepad1.a;

// Check for rising edge (button pressed now, but was not pressed last loop)
        if (aCurr && !aPrev) {
            arcadeDriveSys.turnDegrees(90, 0.4); // trigger turn once
        }

// Save current state for next loop
        aPrev = aCurr;


        if(MovCurr && !movPrev){
            arcadeDriveSys.moveFowardInchesAtPower(24, 0.5);

        }

        movPrev = MovCurr;

        telemetry.addData("FL ticks", arcadeDriveSys.getFLPosition());
        telemetry.addData("FR ticks", arcadeDriveSys.getFRPosition());
        telemetry.addData("RL ticks", arcadeDriveSys.getRLPosition());
        telemetry.addData("RR ticks", arcadeDriveSys.getRRPosition());

        telemetry.update();
    }
}
