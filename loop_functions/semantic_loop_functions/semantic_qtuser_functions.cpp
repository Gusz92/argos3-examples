#include "semantic_qtuser_functions.h"
#include "semantic_loop_functions.h"

/****************************************/
/****************************************/

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/
void CTrajectoryQTUserFunctions::Init(TConfigurationNode& t_tree)
{

}
/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CTrajectoryLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }

   CSpace::TMapPerType& tCYLMap = m_cTrajLF.GetSpace().GetEntitiesByType("cylinder");

   for(CSpace::TMapPerType::iterator it = tCYLMap.begin();
       it != tCYLMap.end();
       ++it) {
      CCylinderEntity* pcCYL = any_cast<CCylinderEntity*>(it->second);
       CVector3 cylinderpos = pcCYL->GetEmbodiedEntity().GetOriginAnchor().Position;
/*       DrawTriangle(cylinderpos,
                    CQuaternion(CRadians::ZERO, CVector3::Z),
       0.5,
       0.5,
                    CColor::BLACK,
       false);*/
      DrawCircle(cylinderpos,
                 CQuaternion(CRadians::ZERO, CVector3::Z),
                 0.8,
                 CColor::BLACK,
                 false,
                 20);
/*      DrawPoint(cylinderpos,
         CColor::BLACK,
         50);*/
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]),
                 CColor::RED,
                 10.0f);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "semantic_qtuser_functions")
