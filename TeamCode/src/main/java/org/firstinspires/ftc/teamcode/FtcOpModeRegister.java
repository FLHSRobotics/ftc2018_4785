package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;


/**
 * {@link FtcOpModeRegister} is responsible for registering opmodes for use in an FTC game.
 * @see #register(OpModeManager)
 */
public class FtcOpModeRegister implements OpModeRegister{

    @OpModeRegistrar
    public void register(OpModeManager manager) {
         manager.register("Iterative Opmode",BasicOpMode_Iterative.class);
         manager.register("Linear Opmode", BasicOpMode_Linear.class);
    }
}
