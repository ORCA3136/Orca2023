package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.Drive;

public class SpinAuto extends SequentialCommandGroup {
  private static final double drivePercent = 0.5;
  private static final double duration = 2;

  /** Creates a new SpinAuto, which spins in place for ten seconds. */
  public SpinAuto(Drive drive) {
    addCommands(
        new StartEndCommand(() -> drive.drivePercent(drivePercent, -drivePercent), drive::stop, drive)
            .withTimeout(duration));
        new StartEndCommand(() -> drive.drivePercent(-drivePercent, drivePercent), drive::stop, drive)
            .withTimeout(duration);
        
  }
}
