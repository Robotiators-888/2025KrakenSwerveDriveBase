package frc.robot.subsystems;

import java.util.logging.Logger;
import frc.robot.subsystems.GyroIO.GyroIOInputs;
import frc.robot.subsystems.ModuleIO.ModuleIOInputs;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputs inputs = new ModuleIOInputs(); // was called ModuleIOInputsAutoLog a temple for advantage kit, will change the name if nessary
    private final int index;
    
    // public ModuleIO(GyroIO gyroIO,  //needs a return message
    //  ModuleIO moduleIO) {
    // this.io = moduleIO;
    // this.gyroIO = gyroIO;
    // }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        }
}
