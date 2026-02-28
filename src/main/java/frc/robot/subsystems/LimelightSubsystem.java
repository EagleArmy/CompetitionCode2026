// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionProfile;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private Debouncer alignmentDebouncer = new Debouncer(0.2);
  
  /* Creates a new Vision. */
  public LimelightSubsystem() {
    setLimelightPipeline("limelight", 0);
  }

  /**
   * Sets commanded pipeline setup to limelight.
   * @param limelight String, limelight name
    *@param pipeline int, commanded pipeline
   */

  private void setLimelightPipeline(String limelight, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelight, pipeline);
  }
  
  
   /* Retrieves reef Tx from left reef pipelines.
   *
   * @param limelight String, front limelight
   * @return double, limelight Tx
   */
  
public double getHubTA(String Limelight){
  if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
    setLimelightPipeline(Limelight, 0);
    return LimelightHelpers.getTY(Limelight);
  }
  else{
    setLimelightPipeline(Limelight, 0);
    return LimelightHelpers.getTY(Limelight);
  }
}



 public double getCenterReefTx(String limelight) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      setLimelightPipeline(limelight, VisionProfile.blueReefCenterPipeline);

      return -LimelightHelpers.getTX(limelight);
      
    }
    else {
      setLimelightPipeline(limelight, VisionProfile.redReefCenterPipeline);
      return -LimelightHelpers.getTX(limelight);
    }
  }


  public boolean limelightAlignedTx() {
    if ((LimelightHelpers.getTX("limelight") <= 0.8) && (LimelightHelpers.getTX("limelight") >= -0.8)) {
      return alignmentDebouncer.calculate(true);
    }
    else {
      return false;
    }
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    SmartDashboard.putBoolean("AlignedWithhubX", limelightAlignedTx());
  }
}