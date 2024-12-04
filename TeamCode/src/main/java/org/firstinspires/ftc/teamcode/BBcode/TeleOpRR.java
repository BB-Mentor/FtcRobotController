package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ClawActions;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpRR")
public class TeleOpRR extends OpMode {
    private List<Action> runningActions = new ArrayList<>();
    ClawActions _ClawActions;

    @Override
    public void init() {
        _ClawActions = new ClawActions(this);
    }

    @Override
    public void loop() {


        //KEEP AT BOTTOM OF LOOP
        //updates and runs RR actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(new TelemetryPacket())) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }
}
