/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_SEMANTIC_H
#define FOOTBOT_SEMANTIC_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
//time.h to record rotate time
#include <time.h>
//positioning sensor
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotSemantic : public CCI_Controller {
public:
    struct SWheelTurningParams {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum ETurningMechanism {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
    * Angular thresholds to change turning state.
    */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    //The state struct of footbot semantic
    struct SStateData{
        enum EState {
            WANDER = 0,
            APPROACH_BLOB,
            ROTATE_BLOB,
            LEAVE_BLOB
        } State;

        void Init(void);
        void Reset();
    };

    struct SDiffusionParams {
        /*
         * Maximum tolerance for the proximity reading between
         * the robot and the closest obstacle.
         * The proximity reading is 0 when nothing is detected
         * and grows exponentially to 1 when the obstacle is
         * touching the robot.
         */
        Real Delta;
        /* Angle tolerance range to go straight. */
        CRange<CRadians> GoStraightAngleRange;

        /* Constructor */
        SDiffusionParams();

        /* Parses the XML section for diffusion */
        void Init(TConfigurationNode& t_tree);
    };

    //the node struct of obstacles
    struct SObstacleNode{
        CColor BlobColor;
        CVector2 BlobPosition;
        clock_t Rotatetime;
        clock_t LeaveTime;
    };
public:
     /* Class constructor. */
   CFootBotSemantic();

   /* Class destructor. */
   virtual ~CFootBotSemantic() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

protected:
    /*
 * Calculates the vector to the closest light.
 */
    virtual CVector2 VectorToBlob();

    CVector2 DiffusionVector(bool& b_collision);

    void SetWheelSpeedsFromVector(const CVector2& c_heading);

    void Wander();
    void ApproachBlob();
    void RotateBlob();
    void LeaveBlob();

    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* TargetBlob(void);
    //Calculate the pos of obstacle nodes
    CVector2 GetBlobPos(const  CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* p_Blob);
    //Check whether the Blob belong to Blob List
    inline bool CheckBlob(const  CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* p_Blob){
        CVector2 BlobPos = GetBlobPos(p_Blob);
        for(std::vector<SObstacleNode>::const_iterator blob_iter=BlobObstacleList.begin(); blob_iter != BlobObstacleList.end(); blob_iter++){
            if(Abs((*blob_iter).BlobPosition.GetX() - BlobPos.GetX()) < 0.3) {
                if (Abs((*blob_iter).BlobPosition.GetY() - BlobPos.GetY()) < 0.3) {
                    if (p_Blob->Color == (*blob_iter).BlobColor)
                        return true;
                }
            }
        }
        return false;
    };
private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;

   // CCI_FootBotLightSensor* m_pcLight;

    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcColoredBlob;
    CCI_PositioningSensor* m_pcPosition;
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The controller state information */
    SStateData m_sStateData;
    /* The diffusion parameters */
    SDiffusionParams m_sDiffusionParams;

    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* p_TargetBlob;
    //vector list of obstacle nodes
    std::vector<SObstacleNode> BlobObstacleList;
    //time for recording the start of rotate or leaving
    clock_t startTime;
};

#endif
