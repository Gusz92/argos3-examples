#ifndef SEMANTIC_QTUSER_FUNCTIONS_H
#define SEMANTIC_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CTrajectoryLoopFunctions;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrajectoryQTUserFunctions();

   virtual ~CTrajectoryQTUserFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);

   virtual void DrawInWorld();

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CTrajectoryLoopFunctions& m_cTrajLF;

};

#endif
