package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class GyroIOReal implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOReal() {
        pigeon = new Pigeon2(DriveConstants.kPigeon2Id, DriveConstants.kCANbusName);
        pigeon.setYaw(0.0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // 檢查陀螺儀有沒有斷線
        inputs.connected = pigeon.isConnected();
        
        // 🟢 取得 YAW 角度 (車頭朝向) - 取代舊的 getAngle()
        inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValueAsDouble());
        
        // 🟢 取得 Z 軸旋轉角速度 - 取代舊的 getRate()
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}