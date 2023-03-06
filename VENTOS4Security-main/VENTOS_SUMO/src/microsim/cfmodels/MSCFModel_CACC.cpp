
// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_CACC.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>

//included by Rohit
#include <traci-server/TraCIServerAPI_Vehicle.h>
#include <utils/traci/TraCIAPI.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
using namespace std;
#include <map>

using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::ostringstream;
using std::istringstream;

// mani
#include <math.h>
#include <microsim/MSNet.h> // for getting the current simulation time

// ===========================================================================
// method definitions
// ===========================================================================

MSCFModel_CACC::MSCFModel_CACC(const MSVehicleType* vtype,
        int myStrategy,
        SUMOReal MaxAccel,
        SUMOReal MaxDecel,
        SUMOReal tau,
        SUMOReal delay,
        SUMOReal ComfAccel,
        SUMOReal ComfDecel,
        SUMOReal K_sc,
        SUMOReal K_v,
        SUMOReal K_d,
        SUMOReal K_a,
        SUMOReal V_int,
        SUMOReal K_v_f,
        SUMOReal K_g_f,
        bool degradeToACC,
        SUMOReal invalidTimer) : MSCFModel_ACC(vtype, MaxAccel, MaxDecel, tau, delay, ComfAccel, ComfDecel, K_sc, K_v, K_d, V_int)
{
    this->myStrategy = myStrategy;
    this->myK_a = K_a;
    this->myK_v_f = K_v_f;
    this->myK_g_f = K_g_f;
    this->degradeToACC = degradeToACC;
    this->invalidTimer = invalidTimer;
}


MSCFModel_CACC::~MSCFModel_CACC() {}


SUMOReal
MSCFModel_CACC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const 
{
    // note 1: when this method is called, it means we definitely have a preceding car.         
    // note 2: we should not use variable 'a' directly from SUMO; we need to get it from OMNET++

    // modify vehicle parameters through vehAccess
    SUMOVehicle* sumoVehicle = MSNet::getInstance()->getVehicleControl().getVehicle(veh->getID());
    MSVehicle* vehAccess = dynamic_cast<MSVehicle*>(sumoVehicle);

    if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer1 [900];
        sprintf (buffer1, "\n--------\n%s\n--------", vehAccess->getID().c_str());
        WRITE_MESSAGE(buffer1);
    }

    // make sure I am part of a platoon
    if(vehAccess->myPlatoonView.platoonId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' is not part of any platoon. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }

        return 0;
    }

    // make sure platoon size is valid
    if(vehAccess->myPlatoonView.platoonSize <= 0)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' has an invalid platoon size of '%d'. Follow speed is set to ZERO \n", vehAccess->getID().c_str(), vehAccess->myPlatoonView.platoonSize);
            WRITE_WARNING(buffer);
        }

        return 0;
    }

    // make sure my platoon index is valid
    if(vehAccess->myPlatoonView.platoonDepth < 0 || vehAccess->myPlatoonView.platoonDepth >= vehAccess->myPlatoonView.platoonSize)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' has an invalid platoon depth of '%d'. Follow speed is set to ZERO \n", vehAccess->getID().c_str(), vehAccess->myPlatoonView.platoonDepth);
            WRITE_WARNING(buffer);
        }

        return 0;
    }

    // platoon leader always falls back to ACC
    if(vehAccess->myPlatoonView.platoonDepth == 0)
    {
        if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "Platoon leader '%s' is falling back to ACC mode \n", vehAccess->getID().c_str());
            WRITE_MESSAGE(buffer);
        }

        return switchToACC(vehAccess, speed, gap, predSpeed, predMaxDecel); //Rohit : myswitchToACC() or myfollowSpeed() or leadercontroller()
    }

    // CACC with one-vehicle look-ahead communication
    if(myStrategy == 1)
        return controller_1(vehAccess, speed, gap, predSpeed, predMaxDecel);
    // CACC with acceleration from platoon leader
    else if(myStrategy == 2)
        return controller_2(vehAccess, speed, gap, predSpeed, predMaxDecel);
    // CACC with bi-directional control
    else if(myStrategy == 3)
        return controller_3(vehAccess, speed, gap, predSpeed, predMaxDecel);
    else
        throw ProcessError("Invalid 'strategy' for CACC vehicle! \n");
}


double
MSCFModel_CACC::controller_1(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const
{
   vehAccess->caller_getSafeFollowSpeed = false;
    // get the name of the front vehicle
    int frontVehDepth = vehAccess->myPlatoonView.platoonDepth - 1;
    typedef std::map<int, platoonConfig_t>::iterator config_i;
    config_i ii = vehAccess->myPlatoonView.platoonConfiguration.find(frontVehDepth);

    if(ii == vehAccess->myPlatoonView.platoonConfiguration.end() || ii->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the front vehicle of '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }

    platoonConfig_t frontVehConfig = ii->second;
    // written by Rohit
    /*
    int mydepth = vehAccess->myPlatoonView.platoonDepth;
    typedef std::map<int, platoonConfig_t>::iterator config_imy;
    config_imy iimy = vehAccess->myPlatoonView.platoonConfiguration.find(mydepth);

    if(iimy == vehAccess->myPlatoonView.platoonConfiguration.end() || iimy->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the my vehicle having id '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }
    platoonConfig_t myvehconfig = iimy->second;
    */
    /*
    if(iimy == vehAccess->myPlatoonView.platoonConfiguration.end() || iimy->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the front vehicle of '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }*/
    

    // check if the data from the front vehicle is fresh
    //int rc = checkPlatoonConfigTimestamp(vehAccess, frontVehConfig);
    //Rohit : commenting if block
/*
    if(rc == -1)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' is downgrading to to ACC mode \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }

        return switchToACC(vehAccess, speed, gap, predSpeed, predMaxDecel);
    }
    */

    if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer1 [900];
        sprintf (buffer1, "cfmodel : Intelligent connected vehicles with two vehicles."); //Rohit : "Controller: CACC with one-vehicle look-ahead communication"
        WRITE_MESSAGE(buffer1);
    }

    // Rohit : Commenting gap equation
    // the gap in argument excludes minGap. We will add the minGap
    //gap = gap + ; //Rohit : gap = gap + vehAccess->getVehicleType().getMinGap()

    // Rohit : commenting if block
    // apply measurement error
    //if(vehAccess->errorGap != 0 || vehAccess->errorRelSpeed != 0)
    //    applyMeasurementError(vehAccess, &gap, speed, &predSpeed);

    // Rohit : commenting if block because i don't care debugging
    // printing input values (for debugging purposes)
    /*if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer [900];
        sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f",
                speed,
                vehAccess->getAcceleration(),
                gap);
        WRITE_MESSAGE(buffer);

        char buffer2 [900];
        sprintf (buffer2, "Parameters of the front vehicle '%s': speed=%.3f, accel(wireless)=%.3f, maxDecel=%.3f",
                frontVehConfig.vehId.c_str(),
                predSpeed,
                frontVehConfig.accel,
                predMaxDecel);
        WRITE_MESSAGE(buffer2);
    }*/

    //Rohit : commenting emergency situation actions
    //int result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
    // we need emergency break!
    //if(result != -1)
    //    return result;

    // Rohit : commenting because we don't want any intended velocity (v_int)
    // V_int should not be bigger than 'maximum lane speed' or 'maximum vehicle speed'
    //SUMOReal myV_int2 = MIN3( myV_int, vehAccess->getLane()->getVehicleMaxSpeed(vehAccess), vehAccess->getVehicleType().getMaxSpeed() );

    //Rohit : commenting because we don't want to control speed but we want to control gap
    // desired accel for speed control
    //SUMOReal a_des_v = myK_sc * (myV_int2 - speed);

    // Rohit : commenting
    // desired accel for gap control
    //SUMOReal a_des_g = myK_a * frontVehConfig.accel +
            //myK_v * (predSpeed - speed) +
            //myK_g * (gap - vehAccess->getVehicleType().getMinGap() - (speed*myHeadwayTime));

    //SUMOReal a_control = MIN2(a_des_v, a_des_g);

    // get current acceleration of vehicle
    //SUMOReal a = vehAccess->getAcceleration();

    //SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;

    // bound a_des to [-ComfDecel, ComfAccel]
    //SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    
/*    SUMOReal frontVehAccel[600];
    for(int i = 0; i < 150; i++)
        frontVehAccel[i] = 3.5;
    for(int i = 150; i < 300; i++)
        frontVehAccel[i] = 0;
    for(int i = 300; i < 450; i++)
        frontVehAccel[i] = 3.5;
    for(int i = 450; i < 600; i++)
        frontVehAccel[i] = 0;
*/

    /*
    std::string str;
            
    if (!server.readTypeCheckingString(inputStorage, str)) 
    {
        return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "vehiclePlatoonViewUpdate requires a string.", outputStorage);
    }
    std::stringstream timeStamp_str;
    
    for(int i = 0; i < str.length(); i++)
    {
        if(str[i] == '#')
        {
            count++;
            continue;
        }
        
        if(count == 0)
            timeStamp_str << str[i];
    }           

    if(count != 7)
        return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "vehiclePlatoonViewUpdate expects 7 parameters.", outputStorage);

    double timeStamp = atof(timeStamp_str.str().c_str());
    */

    /*
    std::string depart;
    depart = toString(myParent.simulation.getCurrentTime() / 1000.0);
    */

    
    int depart;
    depart = MSNet::getInstance()->getCurrentTimeStep();
    

    // safety attack vector creating accident 
    SUMOReal attack[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*
    // Bad performance attack vector causing traffic
    SUMOReal attack[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-4,-4,-4,-4.875,-4.9375,-4.9921875,-4.999755859,-4.99987793,-4.999984741,-4.999992371,-4.999998093,-4.999999985,-4.999999996,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    */
    /*
    string filename("falsedata_abrupt_2.csv");
    string file_contents;

    auto ss = ostringstream{};
    ifstream input_file(filename);
    if (!input_file.is_open()) {
        cerr << "Could not open the file - '"
             << filename << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    file_contents = ss.str();
    */

    /*
    //UNABLE TO OPEN FILE EXAMPLE
    char huha[900];
    string line;
  ifstream myfile("falsedata_abrupt_2.csv");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
        sprintf(huha, "chhapo : %s \n", line);
        WRITE_MESSAGE(huha);
    }
    myfile.close();
  }
  else
  {
    sprintf(huha, "Unable to open file", line);
    WRITE_MESSAGE(huha);
  }
  */

    SUMOReal falseaccel = vehAccess->getAcceleration();

    char buffer2 [900];
        sprintf (buffer2, "time = %d, vehid = %d, frontvehaccel = %.3f, veh1 pos = %.3f, veh1 vel = %.3f, veh1 accel = %.3f, veh2 pos = %.3f, veh2 vel = %.3f, veh2 accel = %.3f \n",
                depart/100,
                vehAccess->myPlatoonView.platoonDepth,
                frontVehConfig.accel,
                vehAccess->getPositionOnLane()+gap,
                predSpeed,
                frontVehConfig.accel,
                vehAccess->getPositionOnLane(),
                speed,
                vehAccess->getAcceleration());                     //, myvehconfig.accel , edge.getAccel("veh2"), vehAccess->getAcceleration(), falseaccel   
        WRITE_MESSAGE(buffer2);



    if(vehAccess->myPlatoonView.platoonDepth == 3)
    {
        // adversary
        frontVehConfig.accel += attack[depart/100];

    }
    


    SUMOReal u = 0.05*(vehAccess->getPositionOnLane()+gap)+0.3*predSpeed+0.98*frontVehConfig.accel-0.05*vehAccess->getPositionOnLane()-0.325*speed-0.15*falseaccel-0.2;
    SUMOReal a_matrix[3][3] = {{1,0.1,0.005},
                                {0,1,0.1},
                                {0,0,0}};
    SUMOReal b_matrix[3] = {0,0,1};
    SUMOReal x_state[3] = {vehAccess->getPositionOnLane(),speed,falseaccel};
    for(int i = 2; i >= 0; i--)
    {
        SUMOReal x = 0;
        for(int j = 0; j < 3; j++)
            x += a_matrix[i][j]*x_state[j];
        x_state[i] = x + b_matrix[i]*u; // frontVehAccel[MSNet::getInstance()->getCurrentTimeStep()]
    }
    SUMOReal followV = x_state[1];
/*
    SUMOReal a_matrix[7][7] = {{1,0.1,0.005,0,0,0,0},
                               {0,1,0.1,0,0,0,0},
                               {0,0,0,0,0,0,0},
                               {0,0,0,1,0.1,0.005,0},
                               {0,0,0,0,1,0.1,0},
                               {0.05,0.3,0.98,-0.05,-0.325,-0.15,1},
                               {0,0,0,0,0,0,1}};

    SUMOReal b_matrix[7] = {0,0,1,0,0,0,0};
    SUMOReal x_state[7] = {vehAccess->getPositionOnLane()+gap,predSpeed,frontVehConfig.accel,vehAccess->getPositionOnLane(),speed,vehAccess->getAcceleration(),-0.2};
    
    for(int i = 6; i >= 0; i--)
    {
        SUMOReal x = 0;
        for(int j = 0; j < 7; j++)
            x += a_matrix[i][j]*x_state[j];
        x_state[i] = x + b_matrix[i]*frontVehConfig.accel; // frontVehAccel[MSNet::getInstance()->getCurrentTimeStep()]
    }
    */

    /*sprintf (buffer2, "veh %d : time = %.3f, pos = %.3f, vel = %.3f, accel = %.3f\n",
                vehAccess->myPlatoonView.platoonDepth,
                MSNet::getInstance()->getCurrentTimeStep(),
                x_state[0],
                x_state[1],
                x_state[2]);
        WRITE_MESSAGE(buffer2);*/
    
    //SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    //SUMOReal followV = x_state[4];

    // make sure followV is not negative
    followV = MAX2(0.,followV);

/*
    // printing output values (for debugging purposes)
    if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer2 [900];
        sprintf (buffer2, "Output values: a_veh=%.3f, followV=%.3f \n",
                x_state[5],
                followV);
        WRITE_MESSAGE(buffer2);
    }
*/
    
    

    if(followV == 0)
        vehAccess->myCFMode = Mode_Stopped;
    else
        vehAccess->myCFMode = Mode_SpeedControl;

    return followV;
}


double
MSCFModel_CACC::controller_2(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const
{
    
    vehAccess->caller_getSafeFollowSpeed = false;
    // get the name of the front vehicle
    int frontVehDepth = vehAccess->myPlatoonView.platoonDepth - 1;
    typedef std::map<int, platoonConfig_t>::iterator config_i;
    config_i ii = vehAccess->myPlatoonView.platoonConfiguration.find(frontVehDepth);

    if(ii == vehAccess->myPlatoonView.platoonConfiguration.end() || ii->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the front vehicle of '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }

    platoonConfig_t frontVehConfig = ii->second;
    // written by Rohit

    config_i jj = vehAccess->myPlatoonView.platoonConfiguration.find(0);
    if(jj == vehAccess->myPlatoonView.platoonConfiguration.end() || jj->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the platoon leader of vehicle of '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }

        return 0;
    }

    platoonConfig_t platoonLeaderConfig = jj->second;

    // check if the data from the platoon leader is fresh
    int rc = checkPlatoonConfigTimestamp(vehAccess, platoonLeaderConfig);
    if(rc == -1)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' is downgrading to to ACC mode \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }

        return switchToACC(vehAccess, speed, gap, predSpeed, predMaxDecel);
    }






    
    int mydepth = vehAccess->myPlatoonView.platoonDepth;
    typedef std::map<int, platoonConfig_t>::iterator config_imy;
    config_imy iimy = vehAccess->myPlatoonView.platoonConfiguration.find(mydepth);

    if(iimy == vehAccess->myPlatoonView.platoonConfiguration.end() || iimy->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the my vehicle having id '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }
    platoonConfig_t myvehconfig = iimy->second;
    
    
    if(iimy == vehAccess->myPlatoonView.platoonConfiguration.end() || iimy->second.vehId == "")
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "cannot get the front vehicle of '%s'. Follow speed is set to ZERO \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
            return 0;
    }
    

    // check if the data from the front vehicle is fresh
    //int rc = checkPlatoonConfigTimestamp(vehAccess, frontVehConfig);
    //Rohit : commenting if block

    if(rc == -1)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "vehicle '%s' is downgrading to to ACC mode \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }

        return switchToACC(vehAccess, speed, gap, predSpeed, predMaxDecel);
    }
    










    if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer1 [900];
        sprintf (buffer1, "cfmodel : Intelligent connected vehicles with two vehicles."); //Rohit : "Controller: CACC with one-vehicle look-ahead communication"
        WRITE_MESSAGE(buffer1);
    }


    

    // Rohit : Commenting gap equation
    // the gap in argument excludes minGap. We will add the minGap
    //gap = gap + ; //Rohit : gap = gap + vehAccess->getVehicleType().getMinGap()

    // Rohit : commenting if block
    // apply measurement error
    //if(vehAccess->errorGap != 0 || vehAccess->errorRelSpeed != 0)
    //    applyMeasurementError(vehAccess, &gap, speed, &predSpeed);

    // Rohit : commenting if block because i don't care debugging
    // printing input values (for debugging purposes)
    /*if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer [900];
        sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f",
                speed,
                vehAccess->getAcceleration(),
                gap);
        WRITE_MESSAGE(buffer);

        char buffer2 [900];
        sprintf (buffer2, "Parameters of the front vehicle '%s': speed=%.3f, accel(wireless)=%.3f, maxDecel=%.3f",
                frontVehConfig.vehId.c_str(),
                predSpeed,
                frontVehConfig.accel,
                predMaxDecel);
        WRITE_MESSAGE(buffer2);
    }*/

    //Rohit : commenting emergency situation actions
    //int result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
    // we need emergency break!
    //if(result != -1)
    //    return result;

    // Rohit : commenting because we don't want any intended velocity (v_int)
    // V_int should not be bigger than 'maximum lane speed' or 'maximum vehicle speed'
    //SUMOReal myV_int2 = MIN3( myV_int, vehAccess->getLane()->getVehicleMaxSpeed(vehAccess), vehAccess->getVehicleType().getMaxSpeed() );

    //Rohit : commenting because we don't want to control speed but we want to control gap
    // desired accel for speed control
    //SUMOReal a_des_v = myK_sc * (myV_int2 - speed);

    // Rohit : commenting
    // desired accel for gap control
    //SUMOReal a_des_g = myK_a * frontVehConfig.accel +
            //myK_v * (predSpeed - speed) +
            //myK_g * (gap - vehAccess->getVehicleType().getMinGap() - (speed*myHeadwayTime));

    //SUMOReal a_control = MIN2(a_des_v, a_des_g);

    // get current acceleration of vehicle
    //SUMOReal a = vehAccess->getAcceleration();

    //SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;

    // bound a_des to [-ComfDecel, ComfAccel]
    //SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    
/*    SUMOReal frontVehAccel[600];
    for(int i = 0; i < 150; i++)
        frontVehAccel[i] = 3.5;
    for(int i = 150; i < 300; i++)
        frontVehAccel[i] = 0;
    for(int i = 300; i < 450; i++)
        frontVehAccel[i] = 3.5;
    for(int i = 450; i < 600; i++)
        frontVehAccel[i] = 0;
*/

    /*
    std::string str;
            
    if (!server.readTypeCheckingString(inputStorage, str)) 
    {
        return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "vehiclePlatoonViewUpdate requires a string.", outputStorage);
    }
    std::stringstream timeStamp_str;
    
    for(int i = 0; i < str.length(); i++)
    {
        if(str[i] == '#')
        {
            count++;
            continue;
        }
        
        if(count == 0)
            timeStamp_str << str[i];
    }           

    if(count != 7)
        return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "vehiclePlatoonViewUpdate expects 7 parameters.", outputStorage);

    double timeStamp = atof(timeStamp_str.str().c_str());
    */

    /*
    std::string depart;
    depart = toString(myParent.simulation.getCurrentTime() / 1000.0);
    */

    int depart;
    depart = MSNet::getInstance()->getCurrentTimeStep();
    

    // safety attack vector creating accident 
    SUMOReal attack[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.5,4.997558593,4.999923706,4.999990463,4.999995231,4.999997615,4.99999985,4.99999999,4.999999997,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,4.999999999,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};




/*
    // Bad performance attack vector causing traffic
    SUMOReal attack[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-4,-4,-4,-4.875,-4.9375,-4.9921875,-4.999755859,-4.99987793,-4.999984741,-4.999992371,-4.999998093,-4.999999985,-4.999999996,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,-5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    */
    /*
    string filename("falsedata_abrupt_2.csv");
    string file_contents;

    auto ss = ostringstream{};
    ifstream input_file(filename);
    if (!input_file.is_open()) {
        cerr << "Could not open the file - '"
             << filename << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    file_contents = ss.str();
    */

    /*
    //UNABLE TO OPEN FILE EXAMPLE
    char huha[900];
    string line;
  ifstream myfile("falsedata_abrupt_2.csv");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
        sprintf(huha, "chhapo : %s \n", line);
        WRITE_MESSAGE(huha);
    }
    myfile.close();
  }
  else
  {
    sprintf(huha, "Unable to open file", line);
    WRITE_MESSAGE(huha);
  }
  */

    SUMOReal n = 4, tau = 0.5, d = 20, veh_len = 4, rogue = 2, ts = 0.1;
    SUMOReal a[3][3] = {{0,1,0},
                                {0,0,1},
                                {0,0,-1/tau}};
    SUMOReal b[3] = {0,0,1/tau};
    SUMOReal m[4][4] = {{0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0}};

    SUMOReal l[4][4] = {{0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0}};

    SUMOReal p[4][4] = {{0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0}};
    SUMOReal z[3] = {};
    for(int i = 0; i < 4; i++)
    {
        m[i+1][i] = 1;
        p[i][i] = 1;
    }
    p[4][4] = 1;
    char buffer [900];
    sprintf (buffer, "Reached here, execution done");
    WRITE_WARNING(buffer);

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            if(i == j)
            {
                for(int k = 0; k < 5; k++)
                {
                    l[i][j] += m[i][k];
                }
            }
            else
            {
                l[i][j] = (-1)*m[i][j];
            }
        }
    }

    SUMOReal x[3][5] = {};
    for(int i = 0; i <= n; i++)
    {
        x[0][i] = (n-i+1)*(veh_len+d);
        x[1][i] = 20;
        x[2][i] = 0;
    }

    SUMOReal s_0 = x[1][1], v_prev = 20;
    SUMOReal v_0 = platoonLeaderConfig.speed;
    SUMOReal a_0 = (v_0-v_prev)/0.1;
    s_0 = s_0 + (v_prev*0.1) + (0.5*a_0*0.1*0.1);
    x[0][1] = s_0;
    x[1][1] = v_0;
    x[2][1] = a_0;
    v_prev = v_0;
    SUMOReal k[3] = {1,2,1};

    for(int i = 0; i <= n; i++)
    {
        SUMOReal u = 0;
        if(p[i][i])
        {
            SUMOReal d_i_0 = (i)*d + (i)*veh_len;
            SUMOReal temp[3] = {};
            temp[0] = x[0][i+1]-s_0-d_i_0;
            temp[1] = x[1][i+1]-v_0;
            temp[2] = x[2][i+1]-a_0;
            u = u - k[0]*temp[0]-k[1]*temp[1]-k[2]*temp[2];
        }
        for(int j = 0; j <= i-1; i++)
        {
            if(m[i][j])
            {
                SUMOReal d_i_j = (i-j)*d+(i-j)*veh_len;
                if(vehAccess->myPlatoonView.platoonDepth == j)
                {
                    SUMOReal temp[3] = {};
                    temp[0] = x[0][i+1]-x[0][j+1]-d_i_j;
                    temp[1] = x[1][i+1]-x[1][j+1];
                    temp[2] = x[2][i+1]-x[2][j+1]-attack[depart/100];
                    u = u-k[0]*temp[0]-k[1]*temp[1]-k[2]*temp[2];
                }
                else
                {
                    SUMOReal temp[3] = {};
                    temp[0] = x[0][i+1]-x[0][j+1]-d_i_j;
                    temp[1] = x[1][i+1]-x[1][j+1];
                    temp[2] = x[2][i+1]-x[2][j+1];
                    u = u-k[0]*temp[0]-k[1]*temp[1]-k[2]*temp[2];
                }
            }
        }
        for(int k = 0; k < 3; k++)
        {
            x[k][i+1] = a[k][0]*x[0][i+1]+a[k][1]*x[1][i+1]+a[k][2]*x[2][i+1]+b[k]*u;
        }
        
        z[0] = x[0][i+1];
        z[1] = x[1][i+1];
        z[2] = x[2][i+1];
    }

    char buffer2 [900];
        sprintf (buffer2, "time = %d, vehid = %d, frontvehaccel = %.3f, veh1 pos = %.3f, veh1 vel = %.3f, veh1 accel = %.3f, veh2 pos = %.3f, veh2 vel = %.3f, veh2 accel = %.3f \n",
                depart/100,
                vehAccess->myPlatoonView.platoonDepth,
                frontVehConfig.accel,
                vehAccess->getPositionOnLane()+gap,
                predSpeed,
                frontVehConfig.accel,
                vehAccess->getPositionOnLane(),
                speed,
                vehAccess->getAcceleration());                     //, myvehconfig.accel , edge.getAccel("veh2"), vehAccess->getAcceleration(), falseaccel   
        WRITE_MESSAGE(buffer2);    

    SUMOReal followV = z[1];
/*
    SUMOReal a_matrix[7][7] = {{1,0.1,0.005,0,0,0,0},
                               {0,1,0.1,0,0,0,0},
                               {0,0,0,0,0,0,0},
                               {0,0,0,1,0.1,0.005,0},
                               {0,0,0,0,1,0.1,0},
                               {0.05,0.3,0.98,-0.05,-0.325,-0.15,1},
                               {0,0,0,0,0,0,1}};

    SUMOReal b_matrix[7] = {0,0,1,0,0,0,0};
    SUMOReal x_state[7] = {vehAccess->getPositionOnLane()+gap,predSpeed,frontVehConfig.accel,vehAccess->getPositionOnLane(),speed,vehAccess->getAcceleration(),-0.2};
    
    for(int i = 6; i >= 0; i--)
    {
        SUMOReal x = 0;
        for(int j = 0; j < 7; j++)
            x += a_matrix[i][j]*x_state[j];
        x_state[i] = x + b_matrix[i]*frontVehConfig.accel; // frontVehAccel[MSNet::getInstance()->getCurrentTimeStep()]
    }
    */

    /*sprintf (buffer2, "veh %d : time = %.3f, pos = %.3f, vel = %.3f, accel = %.3f\n",
                vehAccess->myPlatoonView.platoonDepth,
                MSNet::getInstance()->getCurrentTimeStep(),
                x_state[0],
                x_state[1],
                x_state[2]);
        WRITE_MESSAGE(buffer2);*/
    
    //SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    //SUMOReal followV = x_state[4];

    // make sure followV is not negative
    //followV = MAX2(0.,followV);

/*
    // printing output values (for debugging purposes)
    if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
    {
        char buffer2 [900];
        sprintf (buffer2, "Output values: a_veh=%.3f, followV=%.3f \n",
                x_state[5],
                followV);
        WRITE_MESSAGE(buffer2);
    }
*/
    
    

    if(followV == 0)
        vehAccess->myCFMode = Mode_Stopped;
    else
        vehAccess->myCFMode = Mode_SpeedControl;


    return followV;
}


double
MSCFModel_CACC::controller_3(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const
{
    //    // we need these two parameters for my following vehicle
    //    SUMOReal followSpeed = -1;
    //    SUMOReal followGap = -1;
    //
    //    // get the following vehicle
    //    const MSVehicle* following = veh->followVeh.vehiclePtrSumo;
    //    // make sure, I have a follower
    //    if(following != NULL)
    //    {
    //        followSpeed = following->getSpeed();
    //        followGap = veh->getBackPositionOnLane(following->getLane()) - following->getPositionOnLane() - following->getVehicleType().getMinGap();
    //    }
    //
    //    // check if we have received all the required data from platoon leader
    //    int result = allDataReceived(vehAccess, vehAccess->platoonLeaderVeh, speed, gap, predSpeed, predMaxDecel);
    //    // switching to ACC
    //    if(result == -1)
    //        return MSCFModel_ACC::followSpeed(veh, speed, gap, predSpeed, predMaxDecel);
    //
    //    if(veh->debug)
    //    {
    //        char buffer1 [900];
    //        sprintf (buffer1, "--------\n%s\n--------\nController: %s",
    //                          veh->getID().c_str(),
    //                          "CACC with bi-directional control");
    //        WRITE_MESSAGE(buffer1);
    //    }
    //
    //    // the gap in argument excludes minGap
    //    // we will add the minGap
    //    gap = gap + veh->getVehicleType().getMinGap();
    //
    //    // apply measurement error
    //    if(veh->errorGap != 0 || veh->errorRelSpeed != 0)
    //        applyMeasurementError(vehAccess, &gap, speed, &predSpeed);
    //
    //    // printing input values (for debugging purposes)
    //    if(veh->debug)
    //    {
    //        // my own parameters
    //        char buffer [900];
    //        sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f",
    //                         speed,
    //                         veh->getAcceleration(),
    //                         gap);
    //        WRITE_MESSAGE(buffer);
    //
    //        // my preceding
    //        char buffer2 [900];
    //        sprintf (buffer2, "Parameters of preceding vehicle %s: speed=%.3f, maxDecel=%.3f",
    //                          veh->precedingVeh.vehiclePtrSumo->getID().c_str(),
    //                          predSpeed,
    //                          predMaxDecel);
    //        WRITE_MESSAGE(buffer2);
    //
    //        // my following
    //        if(veh->followVeh.vehiclePtrSumo != NULL)
    //        {
    //            char buffer3 [900];
    //            sprintf (buffer3, "Parameters of following vehicle %s: speed=%.3f, gap=%.3f",
    //                              veh->followVeh.vehiclePtrSumo->getID().c_str(),
    //                              veh->followVeh.vehiclePtrSumo->getSpeed(),
    //                              followGap);
    //            WRITE_MESSAGE(buffer3);
    //        }
    //        else
    //        {
    //            char buffer3 [900];
    //            sprintf (buffer3, "I have no following vehicles!");
    //            WRITE_MESSAGE(buffer3);
    //        }
    //
    //        // platoon leader parameters
    //        char buffer4 [900];
    //        sprintf (buffer4, "Parameters of platoon leader %s: accel(wireless)=%.3f",
    //                          veh->platoonLeaderVeh.vehicleNameOmnet.c_str(),
    //                          veh->platoonLeaderVeh.accelOmnet);
    //        WRITE_MESSAGE(buffer4);
    //    }
    //
    //    result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
    //    // we need emergency break!
    //    if(result != -1)
    //        return result;
    //
    //    // V_int should not be bigger than 'maximum lane speed' and 'maximum vehicle speed'
    //    SUMOReal myV_int2 = MIN3( myV_int, vehAccess->getLane()->getVehicleMaxSpeed(vehAccess), vehAccess->getVehicleType().getMaxSpeed() );
    //
    //    // desired accel for speed control
    //    SUMOReal a_des_v = myK_sc * (myV_int2 - speed);
    //
    //    // desired accel for gap control
    //    SUMOReal a_des_g;
    //
    //    // if we have a follower
    //    if(vehAccess->followVeh.vehiclePtrSumo != NULL)
    //    {
    //        a_des_g = myK_a * receivedAccel +
    //                  myK_v * (predSpeed - speed) +
    //                  myK_g * ( gap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) ) +
    //                  myK_v_f * (followSpeed - speed) -
    //                  myK_g_f * (followGap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) );
    //    }
    //    else
    //    {
    //        a_des_g = myK_a * receivedAccel +
    //                  myK_v * (predSpeed - speed) +
    //                  myK_g * (gap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) );
    //    }
    //
    //    SUMOReal a_control = MIN2(a_des_v, a_des_g);
    //
    //    // get current acceleration of vehicle
    //    SUMOReal a = vehAccess->getAcceleration();
    //
    //    SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;
    //
    //    // bound a_des to [-ComfDecel, ComfAccel]
    //    SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    //
    //    SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    //
    //    // make sure followV is not negative
    //    followV = MAX2(0.,followV);
    //
    //    // printing output values (for debugging purposes)
    //    if(vehAccess->debug)
    //    {
    //        char buffer2 [900];
    //        sprintf (buffer2, "Output values: a_des_v=%.3f, a_des_g=%.3f, a_des=%.3f, a_veh=%.3f, followV=%.3f",
    //                          a_des_v,
    //                          a_des_g,
    //                          a_des,
    //                          a_veh,
    //                          followV);
    //        WRITE_MESSAGE(buffer2);
    //    }
    //
    //    if(followV == 0)
    //        vehAccess->myCFMode = Mode_Stopped;
    //    else if(a_des_g < a_des_v)
    //        vehAccess->myCFMode = Mode_GapControl;
    //    else
    //        vehAccess->myCFMode = Mode_SpeedControl;
    //
    //    return followV;
}


// -1: switch to ACC
//  1: continue
int
MSCFModel_CACC::checkPlatoonConfigTimestamp(MSVehicle* vehAccess, platoonConfig_t &vehConfig) const
{
    // get current simulation time step
    SUMOTime simTime = MSNet::getInstance()->getCurrentTimeStep();

    // no data from the front vehicle
    // Reason: maybe the front vehicle is not CACC capable
    if(vehConfig.timestamp == -1)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "SimTime=%.2f: vehicle '%s' has not received any beacons from vehicle '%s'",
                    simTime/1000.,
                    vehAccess->getID().c_str(),
                    vehConfig.vehId.c_str());
            WRITE_WARNING(buffer);
        }

        vehAccess->myCFMode = Mode_NoData;

        // we have no choice other than using ACC
        // even if degradeToACC is off
        return -1;
    }

    // if the data from the front vehicle is out-dated
    if( fabs(simTime - vehConfig.timestamp)/1000. > invalidTimer)
    {
        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "SimTime=%.2f: vehicle '%s' has outdated data from vehicle '%s' (invalidTimer=%f) \n"
                    "The last beacon from that vehicle was received at time '%.6f'",
                    simTime/1000.,
                    vehAccess->getID().c_str(),
                    vehConfig.vehId.c_str(),
                    invalidTimer,
                    vehConfig.timestamp/1000.);
            WRITE_WARNING(buffer);
        }

        vehAccess->myCFMode = Mode_DataLoss;

        if(degradeToACC)
            return -1;

        if(vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer2 [900];
            sprintf (buffer2, "vehicle '%s' continue using the CACC controller (Downgrading to ACC is off) \n", vehAccess->getID().c_str());
            WRITE_WARNING(buffer2);
        }

        return 1;
    }

    return 1;
}


void
MSCFModel_CACC::applyMeasurementError(MSVehicle* vehAccess, SUMOReal *gap, SUMOReal speed, SUMOReal *predSpeed) const
{
    if(vehAccess->errorGap != 0)
    {
        SUMOReal oldGap = *gap;

        // measurement error (gap)    
        double r1 = ( ( rand() / double(RAND_MAX) ) - 0.5 ) * 2;     // -1 <= r1 <= 1
        r1 = r1 * ( (vehAccess->errorGap)*100 );                     // -1 <= r1 <= 1    
        *gap = *gap * ( (100 + r1) / 100);

        if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
        {        
            char buffer1 [900];
            sprintf (buffer1, "adding measurement error: errorGap=%.3f, oldGap=%.3f, newGap=%.3f",
                    vehAccess->errorGap,
                    oldGap,
                    *gap);
            WRITE_MESSAGE(buffer1); 
        }
    }

    if(vehAccess->errorRelSpeed != 0)
    {    
        SUMOReal oldPredSpeed = *predSpeed;

        // measurement error (relative speed)
        double r2 = ( ( rand() / double(RAND_MAX) ) - 0.5) * 2;     // -1 <= r2 <= 1
        r2 = r2 * ( (vehAccess->errorRelSpeed)*100 );               // -5 <= r2 <= 5    
        *predSpeed = ( (*predSpeed-speed) * ( (100 + r2) / 100 ) ) + speed; 

        if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
        {        
            char buffer1 [900];
            sprintf (buffer1, "adding measurement error: errorRelSpeed=%.3f, oldPredSpeed=%.3f, predSpeed=%.3f",
                    vehAccess->errorRelSpeed,
                    oldPredSpeed,
                    *predSpeed);
            WRITE_MESSAGE(buffer1); 
        }
    }    
}


int
MSCFModel_CACC::emergencyBrakeNeeded(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel, SUMOReal constant) const
{
    // check if we need to do an emergency break?
    SUMOReal g_safe = SPEED2DIST(speed) - pow(predSpeed,2)/(2*predMaxDecel) + pow(speed,2)/(2*myDecel) + ( vehAccess->getVehicleType().getMinGap() + constant ); 

    if(gap > g_safe)
        return -1;        
    else
    {
        SUMOReal followV = speed - ACCEL2SPEED(myDecel);

        // make sure followV is not negative
        followV = MAX2(0.,followV);

        // printing output values (for debugging purposes)
        if(vehAccess->debug && vehAccess->caller_getSafeFollowSpeed)
        {
            char buffer [900];
            sprintf (buffer, "Output values: (emergency break) followV=%.3f", followV);
            WRITE_MESSAGE(buffer); 
        }

        // update the CFMode
        if(followV == 0)   
            vehAccess->myCFMode = Mode_Stopped;         
        else    
            vehAccess->myCFMode = Mode_EmergencyBrake;

        return followV;        
    }
}


double
MSCFModel_CACC::switchToACC(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const
{
    //    std::string vTypeID = vehAccess->getVehicleType().getID();
    //    std::size_t found = vTypeID.find("@");
    //    if (found != std::string::npos)
    //        vTypeID.erase(found);
    //
    //    // todo: a temporary solution to change parameters
    //    // should change back the parameters once we get out of ACC
    //    MSVehicleType* sumoVehicleType = MSNet::getInstance()->getVehicleControl().getVType(vTypeID);
    //    sumoVehicleType->getCarFollowModel().setHeadwayTime(1.2);
    //    sumoVehicleType->getCarFollowModel().setDelay(0.5);
    //    sumoVehicleType->getCarFollowModel().setK_v(1.0);
    //    sumoVehicleType->getCarFollowModel().setK_g(5.0);

    return MSCFModel_ACC::followSpeed(vehAccess, speed, gap, predSpeed, predMaxDecel);
}

MSCFModel*
MSCFModel_CACC::duplicate(const MSVehicleType* vtype) const 
{
    return new MSCFModel_CACC(vtype, myStrategy, myAccel, myDecel, myHeadwayTime, myDelay, myComfAccel, myComfDecel, myK_sc, myK_v, myK_g, myK_a, myV_int, myK_v_f, myK_g_f, false, invalidTimer);
}
