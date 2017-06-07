/* Include the controller definition */
#include "footbot_semantic.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CFootBotSemantic::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/
void CFootBotSemantic::SStateData::Init(void){

}
/****************************************/
/****************************************/

CFootBotSemantic::SDiffusionParams::SDiffusionParams() :
        GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotSemantic::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotSemantic::CFootBotSemantic() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotSemantic::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   //m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
   m_pcColoredBlob = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor >("colored_blob_omnidirectional_camera");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   TConfigurationNode& diffusion_node = GetNode(t_node, "footbot_diffusion");
   GetNodeAttributeOrDefault(diffusion_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(diffusion_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(diffusion_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   /*
 * Parse the config file
 */
   try {
      /* Wheel turning */
     m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

      m_sStateData.Init();

      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   m_pcColoredBlob->Enable();
}

/****************************************/
/****************************************/

void CFootBotSemantic::ControlStep() {
/*   *//* Get readings from proximity sensor *//*
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   *//* Sum them together *//*
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   *//* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    *//*
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      *//* Go straight *//*
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      *//* Turn, depending on the sign of the angle *//*
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }*/
    switch (m_sStateData.State){
        case SStateData::WANDER: {
            Wander();
            LOG<< "Wander" <<std::endl;
            break;
        }
        case SStateData::APPROACH_BLOB: {
            ApproachBlob();
            LOG<< "Approach" <<std::endl;
            break;
        }
        case SStateData::ROTATE_BLOB: {
            RotateBlob();
            LOG<< "Rotate" <<std::endl;
            break;
        }
        case SStateData::LEAVE_BLOB:{
            LeaveBlob();
            LOG<<"Leave"<<std::endl;
            break;
        }
        default:
            LOGERR << "We can't be here, there's a bug!" << std::endl;
    }


   //RLOG<< m_pcLight->GetReadings()<< std::endl;
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcColoredBlob->GetReadings();
   if(!sReadings.BlobList.empty()) {
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         LOG << "Blob[" << sReadings.BlobList[i]->Color << "]"<< sReadings.BlobList[i]->Distance << sReadings.BlobList[i]->Angle
             << std::endl;
      }
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotSemantic::VectorToBlob() {
   /* Get light readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcColoredBlob->GetReadings();
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   if(!sReadings.BlobList.empty()) {
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
          cAccum = CVector2(sReadings.BlobList[i]->Distance, sReadings.BlobList[i]->Angle);
      }
   }
//   for(size_t i = 0; i < tReadings.size(); ++i) {
//      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
//   }
//   if(cAccum.Length() > 0.0f) {
//       Make the vector long as 1/4 of the max speed
//      cAccum.Normalize();
//      cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
//   }
    cAccum.Normalize();
    cAccum *= 0.75f*m_sWheelTurningParams.MaxSpeed;
   return cAccum;
}

/****************************************/
/****************************************/
CVector2 CFootBotSemantic::DiffusionVector(bool &b_collision){
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X * 0.25*m_sWheelTurningParams.MaxSpeed;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
       cDiffusionVector *= 0.5*m_sWheelTurningParams.MaxSpeed;
      return -cDiffusionVector;
   }
}
/****************************************/
/****************************************/
void CFootBotSemantic::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   /* Turning state switching conditions */
   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
      /* No Turn, heading angle very small */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
      /* Hard Turn, heading angle very large */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else  {
      /* Soft Turn, heading angle in between the two cases */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }

      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }

      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}
/****************************************/
/****************************************/
void CFootBotSemantic::Wander() {
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcColoredBlob->GetReadings();
    bool obs_bool;

    if(!sReadings.BlobList.empty()) {
        m_sStateData.State = SStateData::APPROACH_BLOB;
    }
    else
        SetWheelSpeedsFromVector(DiffusionVector(obs_bool));
}
/****************************************/
/****************************************/
void CFootBotSemantic::ApproachBlob() {
    bool obs_bool;
    SetWheelSpeedsFromVector(VectorToBlob() + DiffusionVector(obs_bool));
    if(obs_bool) {
        startTime = clock(); //Start timer
        m_sStateData.State = SStateData::ROTATE_BLOB;
    }
}
/****************************************/
/****************************************/
void CFootBotSemantic::RotateBlob() {
    bool obs_bool;

    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcColoredBlob->GetReadings();
    /* Calculate a normalized vector that points to the closest light */
    CVector2 cAccum;
    if(!sReadings.BlobList.empty()) {
        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
            cAccum = CVector2(sReadings.BlobList[i]->Distance, sReadings.BlobList[i]->Angle);
        }
    }
    cAccum.Normalize();
    cAccum *= 0.75*m_sWheelTurningParams.MaxSpeed;

    CVector2 cAdd;
    cAdd = cAccum + DiffusionVector(obs_bool);
    cAdd.Rotate(1 * CRadians::PI_OVER_SIX);
    cAdd.operator*=(0.5f);
    SetWheelSpeedsFromVector(cAdd);

    if((clock()-startTime) > 0.6f*CLOCKS_PER_SEC) {
        startTime = clock(); //Start timer
        m_sStateData.State = SStateData::LEAVE_BLOB;
    }
/*    if(!obs_bool || sReadings.BlobList.empty())
        m_sStateData.State = SStateData::WANDER;*/

}
/****************************************/
/****************************************/
void CFootBotSemantic::LeaveBlob()
{
    bool obs_bool;
    SetWheelSpeedsFromVector(DiffusionVector(obs_bool));

    if((clock()-startTime) > 0.5f*CLOCKS_PER_SEC)
        m_sStateData.State = SStateData::WANDER;
}
/****************************************/
/****************************************/
CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* CFootBotSemantic::TargetBlob()
{
   // p_TargetBlob = ;
    return p_TargetBlob;
}
/****************************************/
/****************************************/
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotSemantic, "footbot_semantic_controller")
