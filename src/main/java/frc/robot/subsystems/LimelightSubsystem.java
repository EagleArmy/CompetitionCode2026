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

  public static void setLimelightPipeline(String limelight, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelight, pipeline);
  }
  
  
   /* Retrieves reef Tx from left reef pipelines.
   *
   * @param limelight String, front limelight
   * @return double, limelight Tx
   */
  
  public static boolean TAmove(String Limelight){
  if(LimelightHelpers.getTA(Limelight) <= 0.4 && LimelightHelpers.getTA(Limelight) >= 0.6){
    return false;
  }
  else{
    return true;
  }
}
  public static boolean TAmoveMid(String Limelight){
  if(LimelightHelpers.getTA(Limelight) <= 0.4 && LimelightHelpers.getTA(Limelight) >= 0.6){
    return false;
  }
  else{
    return true;
  }
}
  public static boolean TAmoveFar(String Limelight){
  if(LimelightHelpers.getTA(Limelight) <= 0.4 && LimelightHelpers.getTA(Limelight) >= 0.6){
    return false;
  }
  else{
    return true;
  }
}
//measure values on actual roboto
public double getHubTA(String Limelight){
    setLimelightPipeline(Limelight, 0);
  


  if(LimelightHelpers.getTA(Limelight) < 0.7){
    if(TAmoveFar(Limelight) == true){
      ShooterSubsystem.shooterSpeedTower();
      NeckWheelSubsystem.setNeckWheelSpeed(0);
      if(LimelightHelpers.getTA(Limelight) < 0.5){
    
        return 0.5;
      }
      else{
    
      return -0.5;
    }
 }}
  if(LimelightHelpers.getTA(Limelight) >= 0.7 && LimelightHelpers.getTA(Limelight) < 0.9){
    if(TAmoveMid(Limelight) == true){
      ShooterSubsystem.shooterSpeedTower();
      NeckWheelSubsystem.setNeckWheelSpeed(0);
      if(LimelightHelpers.getTA(Limelight) < 0.5){
    
    return 0.5;
  }
  else{
    
    return -0.5;
  }
}}
  if(LimelightHelpers.getTA(Limelight) >= 0.9){
    if(TAmove(Limelight) == true){
      ShooterSubsystem.shooterSpeedTower();
      NeckWheelSubsystem.setNeckWheelSpeed(0);
      if(LimelightHelpers.getTA(Limelight) < 0.5){
    
         return 0.5;
      }
      else{
    
        return -0.5;
      }
    }
    else{return 0;}
  }
  else{ return 0;}
  }



 public double getHubTx(String limelight) {
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