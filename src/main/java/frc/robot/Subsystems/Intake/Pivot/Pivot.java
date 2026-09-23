package frc.robot.Subsystems.Intake.Pivot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.CancellationException;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.CanCoderConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Pivot extends SubsystemBase{
    protected OverTalonFX pivotMotor;
    protected CANcoder pivotCC;
    private Angle target;

    
    protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    
    public Pivot pivot = new Pivot();

    public Pivot(){

        pivotMotor = new OverTalonFX(PivotConstants.motorConfig(), PivotConstants.motorCanId, Constants.canbus);
        pivotCC = new CANcoder(PivotConstants.CCCanId, Constants.canbus);
        pivotCC.getConfigurator().apply(PivotConstants.CCConfig());
    }
    
    public void setTarget(Angle targetSetter){
            target = targetSetter;
    }

    public Angle getTarget(){
        return target;
    }

    
    public Command setMotionMagic(Angle rotations){
        return
            runOnce(() -> setTarget(rotations))
            .andThen(run(() -> pivotMotor.setControl(motionMagicRequest.withPosition(target))).until(() -> isFinished()));
    }


    public Angle getPosition(){
        return Degrees.of(pivotMotor.getPosition().getValueAsDouble());
    }

    private Angle getError(){
        Angle error = getTarget().minus(getPosition());
        return Degrees.of(Math.abs(error.baseUnitMagnitude()));
    } 

    private boolean isFinished(){
        return getError().baseUnitMagnitude() < PivotConstants.Control.AcceptedError.baseUnitMagnitude();
    }


}
