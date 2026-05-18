package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSys {

    public enum LightState{
        intaking,
        beamBroken,
        shooting,
        far,
        mid,
        close,
        far_intaking,
        mid_intaking,
        close_intaking
    }

    LightState currentState;
    RevBlinkinLedDriver lights;

    public void init(HardwareMap hwmap){
        lights = hwmap.get(RevBlinkinLedDriver.class,"blinken");
    }

    public void setLightState(LightState lightState){
        currentState = lightState;
    }

    public void updateLights(){
        if(currentState == LightState.intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        }else if(currentState == LightState.beamBroken){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else if(currentState == LightState.far){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }else if(currentState == LightState.mid){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }else if(currentState == LightState.close){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else if(currentState == LightState.close_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
        }else if(currentState == LightState.mid_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        }else if(currentState == LightState.far_intaking){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }else if(currentState == LightState.shooting){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER       );
        }else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
    }
}
