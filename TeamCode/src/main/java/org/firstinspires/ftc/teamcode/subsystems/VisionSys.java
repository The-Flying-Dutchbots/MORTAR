package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class VisionSys {
    Limelight3A limelight;


    int firstId = 0;


    public void init(HardwareMap hwmap){
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public int getTagID() {
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            // For AprilTags / Fiducials
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (!fid.isEmpty()) {
                firstId = fid.get(0).getFiducialId();
            }
        }
        return firstId;
    }
}
