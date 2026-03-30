package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import frc.robot.util.roller.Roller;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.RollerConfig.RollerMotor;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOInputsAutoLogged;
import team6230.koiupstream.subsystems.ConditionalAction;
import team6230.koiupstream.subsystems.ExtraIO;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {
    private final RollerIO roller;
    private final RollerIOInputsAutoLogged rollerInputs;

    @AutoLogOutput
    public int step = 1;

    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());

        RollerConfig config = new RollerConfig();
        config.motor = RollerMotor.NEO;
        config.name = "IntakeRoller";
        config.motorId = RobotMap.CanBus.kIntakeRollerID;
        config.gearRatio = IntakeConstants.kRollerGearRatio;

        roller = Roller.makeRollerIO(config);

        rollerInputs = new RollerIOInputsAutoLogged();

        addAnExtraIO(new ExtraIO(roller, rollerInputs, "Roller"));

        addSuperstateBehaviour(RobotState.INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER_AND_INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.SHOOTING_AND_INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.UNJAM, () -> {
            clearConditionalActions();
            roller.runVoltage(-IntakeConstants.kIntakingVolts);
        });
        addSuperstateBehaviour(RobotState.IDLE, () -> {
            clearConditionalActions();
            roller.runVoltage(0);
        });
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER, () -> {
            clearConditionalActions();
            roller.runVoltage(0);
        });
        addSuperstateBehaviour(RobotState.SHOOTING, () -> shooting());
        addSuperstateBehaviour(RobotState.HOME, () -> home());
    }

    private void home() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kClosedAngle);
        roller.runVoltage(0);
    }

    private void intaking() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        registerConditionalAction(new ConditionalAction(
                () -> inputs.relativePivotAngleDeg < IntakeConstants.kMinOpenAngle.getDegrees(),
                () -> roller.runVoltage(IntakeConstants.kIntakingVolts)));
    }

    private void shooting() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        // when the arm is close enough to open we will close it a bit
        registerConditionalAction(new ConditionalAction(
                () -> inputs.relativePivotAngleDeg < IntakeConstants.kMinOpenAngle.getDegrees(), () -> {
                    // we will set the target to the next step
                    indexCycling();
                }));
    }

    private void indexCycling() {
        roller.runVoltage(IntakeConstants.kShootingVolts);
        var angleDeg = IntakeConstants.kMinAngle.getDegrees() + (step * IntakeConstants.kStepSizeDegrees);
        if (angleDeg > IntakeConstants.kMaxAngle.getDegrees()) {
            step = 1;
            angleDeg = IntakeConstants.kMinOpenAngle.getDegrees();
        }
        io.setTargetAngle(Rotation2d.fromDegrees(angleDeg));
        if (angleDeg != IntakeConstants.kMinOpenAngle.getDegrees())
            registerConditionalAction(new ConditionalAction(
                    () -> inputs.relativePivotAngleDeg > inputs.pivotTargetAngle - 3,
                    () -> {
                        step++;
                        indexCycling();
                    }));
        else
            shooting();
    }

    @Override
    public void update() {
        Logger.recordOutput("intake/isConditionsEmpty",
                getConditionalActions().isEmpty());
    }

    @Override
    public boolean isReady() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.INTAKING)
                || Superstate.getInstance().isCurrentWanted(RobotState.HOME))
            return Math.abs(inputs.pivotTargetAngle
                    - inputs.relativePivotAngleDeg) <= IntakeConstants.kErrorToleranceDeg;
        return true;
    }

    @Override
    public IntakeIO getIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new IntakeIORev();

            case SIM:
                return new IntakeIOSim();

            case REPLAY:
                return new IntakeIO() {
                };
        }
        return new IntakeIO() {
        };
    }

    public void setVoltage(double volts) {
        io.runVoltsPivot(volts);
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(inputs.relativePivotAngleDeg);
    }

    public void stopMotor() {
        io.stop();
    }
}