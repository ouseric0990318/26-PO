package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger; 

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    
    // 直接使用剛才在 IntakeIO 裡面手寫的類別，完全跳過對 AutoLogged 類別的依賴
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs(); 

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // 👇 因為你的 inputs 已經實作了 LoggableInputs，這裡就不會再報錯了！
        Logger.processInputs("Intake", inputs); 
    }

    /** 核心控制：以指定電壓運轉滾筒 */
    public void setVoltage(double volts) {
        io.setRollerVoltage(volts);
    }
    public Command runIntakeCommand() {
        // 這裡的 8.0 是電壓值 (Max 12.0)，你可以依照機器實際狀況修改
        return this.run(() -> setVoltage(8.0))
                   .finallyDo(() -> setVoltage(0.0))
                   .withName("RunIntake");
}
}