package frc.robot.util;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorSim {
    double m_CurrentPosition = 0;
    double m_CurrentVelocity = 0;
    boolean m_Inverted = false;
    MotorSim m_OtherMotor;
    double m_Id;

    float m_SoftLimitF;
    float m_SoftLimitR;
    boolean m_SoftLimitFEnabled = false;
    boolean m_SoftLimitREnabled = false;





     public void setCurrentVelocity(double v) {
        m_CurrentVelocity = v;
    }

    public void setCurrentPosition(double x) {
        m_CurrentPosition = x;
    }
    public double getPosition() {
        return this.m_CurrentPosition;
    }
    public double getVelocity() {
        return this.m_CurrentVelocity;
    }


   public MotorSim(int ID, MotorType type) {
    this.m_Id = ID;
    

   }

   public void update(double dt) {
    if (!SoftLimitHit()) {
    if (this.m_OtherMotor != null) {
        this.m_CurrentPosition = this.m_OtherMotor.m_CurrentPosition;
    } else this.m_CurrentPosition += dt*m_CurrentVelocity;
   }
}


  public enum SoftLimitDirection {
    kForward,
    kReverse
  }


  public void setInverted(boolean isInverted) {
  }

  public void setSmartCurrentLimit(int limit) {
  }

  public void setVelocityConversionFactor(double vFactor) {
  }

  public void setPositionConversionFactor(double pFactor) {

  }



public void follow(MotorSim leader) {
    this.follow(leader, true);
}


public void follow(MotorSim leader, boolean x) {
    this.m_OtherMotor = leader;
}
public void set(double speed) {
    setCurrentVelocity(speed);
}




public void setSoftLimit(SoftLimitDirection direction, float limit) {
    if (direction == SoftLimitDirection.kForward) this.m_SoftLimitF = limit;
    else  m_SoftLimitR = limit;
}


public void enableSoftLimit(SoftLimitDirection direction, boolean x) {
   
    if (direction == SoftLimitDirection.kForward) this.m_SoftLimitFEnabled = x;
        else this.m_SoftLimitREnabled = x;
}

public boolean SoftLimitHit() {
return ((this.m_CurrentPosition > this.m_SoftLimitF && this.m_SoftLimitFEnabled && this.m_CurrentVelocity>0) || 
        (this.m_CurrentPosition < this.m_SoftLimitR && this.m_SoftLimitREnabled && this.m_CurrentVelocity<0));
}
















}
