// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_agentsim.h"
#include "multiagent_publicvehicle.h"
#include "multiagent_behavior.h"
#include "multiagent_routesearch.h"
#include "multiagent_gis.h"
#include "multiagent_extension.h"
#include "multiagent_networkinterface.h"
#include "multiagent_tracedefs.h"

#include <random>
#include <unistd.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>

namespace MultiAgent {

#ifdef MULTIAGENT_DEBUG
const bool MultiAgentSimulator::isDebugMode = true;
#else
const bool MultiAgentSimulator::isDebugMode = false;
#endif

EscapeAgentInfo escapeAgentInfoTemp[MAX_N]; //人数分の避難場所情報を一括管理
double congestionTimeList[MAX_SIM];
int agentAddCount = 0;
int agentArrivedCount = 0;
int agentRouteSearchCount = 0;
int spacedSecondCount;
int minAgentId = 0; //最小のAgentIdのAgent情報をescapeAgentInfoTemp[0]に保存するため
int notEscapeAgent = 0;
int abnormalCount = 0;
bool decideFlag = false;    //Agentの避難先がきまっているかどうか(knapsackのみ)
bool moveFlag = false;  //Agentが動き出したかどうか
bool fileFlag = false;  //Current Directoryに読み込みファイルが存在するかどうか
bool countFlag = false; //Agentの情報を取得しているかどうか

bool debugMode = false; //DebugModeの切替変数
bool marginFlag = true;  //marginを避難場所のキャパシティに設定する切替変数
float marginRate = 0.2;   //margin割合
bool randomSeedFlag = false; //Random選択の場合に時間によってSeedが変化する(実効の度に結果が変わる) ※これは初期位置も変化するためRandomの時以外使用しないように
//--------------------------------------------------------------------------------------------------



class AgentMobilityModel : public ObjectMobilityModel {
public:
    AgentMobilityModel(const AgentResource& initResource) : resource(initResource) {}

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime, ObjectMobilityPosition& position) override
    {
        position = resource.MobilityPositionForTime(snapshotTime);
    }
private:
    AgentResource resource;
};

class VehicleMobilityModel : public ObjectMobilityModel {
public:
    VehicleMobilityModel(const AgentIdType& initAgentId) : agentId(initAgentId) {}

    void SetPublicVehicle(const shared_ptr<PublicVehicle>& publicVehiclePtr) {
        vehicleAccessPtr.reset(new PublicVehicleAccess(publicVehiclePtr));
    }
    void SetVehicle(const shared_ptr<Vehicle>& vehiclePtr) {
        vehicleAccessPtr.reset(new VehicleAccess(vehiclePtr, VEHICLE_CAR));
    }
    void SetTaxi(const shared_ptr<Vehicle>& vehiclePtr) {
        vehicleAccessPtr.reset(new VehicleAccess(vehiclePtr, VEHICLE_TAXI));
    }

    VehicleType GetVehicleType() const { return vehicleAccessPtr->GetVehicleType(); }

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position) override
    {
        position = (*this).GetCurrentLocation(snapshotTime);
    }

    ObjectMobilityPosition GetCurrentLocation(const TimeType& currentTime) const {
        const Vertex currentPosition = vehicleAccessPtr->GetPosition();
        const double azimuthDegrees = vehicleAccessPtr->GetDirectionRadians()*(180./PI);

        return ObjectMobilityPosition(
            currentTime,
            currentTime,
            currentPosition.x,
            currentPosition.y,
            currentPosition.z,
            true/*theHeightContainsGroundHeightMeters*/,
            agentId,
            90 - azimuthDegrees);
    }

private:
    class AbstractVehicleAccess {
    public:
        virtual ~AbstractVehicleAccess() {}
        virtual const Vertex& GetPosition() const = 0;
        virtual double GetDirectionRadians() const = 0;
        virtual VehicleType GetVehicleType() const = 0;
    };

    class PublicVehicleAccess : public AbstractVehicleAccess {
    public:
        PublicVehicleAccess(const shared_ptr<PublicVehicle>& initPublicVehiclePtr) : publicVehiclePtr(initPublicVehiclePtr) {}
        virtual const Vertex& GetPosition() const { return publicVehiclePtr->GetPosition(); }
        virtual double GetDirectionRadians() const { return publicVehiclePtr->GetDirectionRadians(); }
        virtual VehicleType GetVehicleType() const { return publicVehiclePtr->GetVehicleType(); }
    private:
        shared_ptr<PublicVehicle> publicVehiclePtr;
    };

    class VehicleAccess : public AbstractVehicleAccess {
    public:
        VehicleAccess(
            const shared_ptr<Vehicle>& initVehiclePtr,
            const VehicleType& initVehicleType)
            :
            vehiclePtr(initVehiclePtr),
            vehicleType(initVehicleType)
        {}

        virtual const Vertex& GetPosition() const { return vehiclePtr->GetPosition(); }
        virtual double GetDirectionRadians() const { return vehiclePtr->GetDirectionRadians(); }
        virtual VehicleType GetVehicleType() const { return vehicleType; }
    private:
        shared_ptr<Vehicle> vehiclePtr;
        VehicleType vehicleType;
    };

    AgentIdType agentId;
    shared_ptr<AbstractVehicleAccess> vehicleAccessPtr;
};

//-------------------------------------------------------------------------------------------------

class AgentNode : public BasicNetworkNode {
public:
    AgentNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const AgentResource& initResource);

    virtual const ObjectMobilityPosition GetCurrentLocation() const {

        const ObjectMobilityPosition position = resource.MobilityPosition();

        if (position.X_PositionMeters() != lastMobilityPosition.X_PositionMeters() ||
            position.Y_PositionMeters() != lastMobilityPosition.Y_PositionMeters() ||
            position.HeightFromGroundMeters() != lastMobilityPosition.HeightFromGroundMeters() ||
            position.AttitudeAzimuthFromNorthClockwiseDegrees() != lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees()) {
            lastMobilityPosition = position;
        }

        return lastMobilityPosition;
    }

private:
    AgentResource resource;
    mutable ObjectMobilityPosition lastMobilityPosition;
};

class VehicleNode : public AgentNode {
public:
    VehicleNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed,
        const shared_ptr<VehicleMobilityModel>& initNodeMobilityModelPtr)
        :
        AgentNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            theGisSubsystemPtr,
            initSimulationEngineInterfacePtr,
            theNodeId,
            runSeed,
            initNodeMobilityModelPtr,
            AgentResource()),
        mobilityModelPtr(initNodeMobilityModelPtr)
    {}

    void SetPublicVehicle(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<PublicVehicle>& publicVehiclePtr) {

        mobilityModelPtr->SetPublicVehicle(publicVehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);

    }
    void SetVehicle(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<Vehicle>& vehiclePtr) {

        mobilityModelPtr->SetVehicle(vehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);
    }
    void SetTaxi(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<Vehicle>& vehiclePtr) {

        mobilityModelPtr->SetTaxi(vehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);
    }

    AgentIdType GetAgentId() const { return (*this).GetNodeId(); }

    VehicleType GetVehicleType() const { return mobilityModelPtr->GetVehicleType(); }

    virtual const ObjectMobilityPosition GetCurrentLocation() const {

        const ObjectMobilityPosition position = mobilityModelPtr->GetCurrentLocation(simulationEngineInterfacePtr->CurrentTime());

        if (position.X_PositionMeters() != lastMobilityPosition.X_PositionMeters() ||
            position.Y_PositionMeters() != lastMobilityPosition.Y_PositionMeters() ||
            position.HeightFromGroundMeters() != lastMobilityPosition.HeightFromGroundMeters() ||
            position.AttitudeAzimuthFromNorthClockwiseDegrees() != lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees()) {
            lastMobilityPosition = position;
        }

        return lastMobilityPosition;
    }

private:
    void EnableMovingObjects(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime) {

        GisSubsystem& gisSubsystem = theAgentGisPtr->GetSubsystem();

        const NodeIdType nodeId = (*this).GetNodeId();

        gisSubsystem.RemoveMovingObject(nodeId);
        gisSubsystem.EnableMovingObjectIfNecessary(
            theParameterDatabaseReader,
            nodeId,
            currentTime,
            mobilityModelPtr);
    }

    shared_ptr<VehicleMobilityModel> mobilityModelPtr;
    mutable ObjectMobilityPosition lastMobilityPosition;
};

inline
AgentNode::AgentNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& theNodeId,
    const RandomNumberGeneratorSeedType& runSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const AgentResource& initResource)
    :
    BasicNetworkNode(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        theNodeId,
        runSeed),
    resource(initResource)
{}

//---------------------------------------------------------------------------------------------------------

bool AStringStartsWith(const string& aLine, const string& aString)
{
    return (aLine.find(aString) == 0);
}
//
const string Agent::modelName = "Mas";


Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr)
    :
    simulatorPtr(initSimulatorPtr),
    theAgentGisPtr(initAgentGisPtr),
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    theRouteSearchSubsystemPtr(initRouteSearchSubsystemPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    currentThreadNumber(MASTER_THREAD_NUMBER),
    originalThreadNumber(0),
    isDeletedAtTheEndOfTimeStep(false),
    hasBicycle(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(ZERO_TIME),
    isTaskInitialized(false),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(NO_ROUTE_NUMBER),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    utility1StatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_Utility1")),
    utility2StatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_Utility2")),
    travelDistanceStatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_TravelDistance")),
    travelTimeStatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_TravelTime")),
    numberNoRouteStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_NoRoutes")),
    numberRouteCalculateTimeStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_RouteCalculationTimes")),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    AgentResource resource(this);

    taskTablePtr->GetInitialLocationId(theAgentGisPtr, homePositionId, resource);

    currentPositionId = homePositionId;

    Vertex position;
    RoadIdType roadId;
    
    //debug by umeki
    //std::cout << "---------Agent class-----" << std::endl;
    Vertex positionU = theAgentGisPtr->GetVertex(currentPositionId.id);
    //std::cout << "GetCurrentPosition::" << GIS_NO << std::endl;

    if (currentPositionId.type == GIS_BUILDING) {
        const Building& building = subsystem.GetBuilding(currentPositionId.id);
        position = building.GetRandomPosition(resource.GetRandomNumberGenerator());

        lastVertexId = building.GetNearestEntranceVertexId(position);
        roadId = building.GetNearestEntranceRoadId(position);

        //debug by umeki
        //std::cout << "Agent(position)_Building:" << position.x <<  ",  " << position.y << std::endl;
        //std::cout << "Agent(lastVertexId)_Building:" << lastVertexId << std::endl;
        //std::cout << "Agent(roadId)_Building:" << roadId << std::endl;
        
                // Umeki part
                // choice initialLocationType = RandomRoad
    }else if (currentPositionId.type == GIS_ROAD){
        
        const Road& road = subsystem.GetRoad(currentPositionId.id);

        //debug by umeki
        //std::cout << "---------currentPositionId=GIS_ROAD---------" << std::endl;
        //std::cout << "Agent.resouce::" << resource.GetRandomNumberGenerator().GenerateRandomDouble()  << std::endl;
        //std::cout << "Road.id::" << road.GetRoadId() << std::endl;
        
        position = road.GetRandomPosition(resource.GetRandomNumberGenerator());
        //std::cout << "Agent(position):" << position.x <<  ",  " << position.y << std::endl;
        lastVertexId = road.GetNearestEntranceVertexId(position);
        //std::cout << "Agent(lastVertexId):" << lastVertexId << std::endl;
        /*  There is not Entrance in Road. So this part is not necesarry
           //roadId = road.GetNearestEntranceRoadId(position);
           //std::cout << "Agent(roadId):" << roadId << std::endl;
        */
        // Umeki write the above
    } else if (currentPositionId.type == GIS_PARK) {

        const Park& park = subsystem.GetPark(currentPositionId.id);

        position = park.GetRandomPosition(resource.GetRandomNumberGenerator());
        lastVertexId = park.GetNearestEntranceVertexId(position);
        roadId = park.GetNearestEntranceRoadId(position);

    //}else if (currentPositionId.type == GIS_NO){
    //    position = resource.Position();
    //    lastVertexId = resource.LastVertexId();
    } else {
        assert(currentPositionId.type == GIS_POI);

        const Poi& poi = subsystem.GetPoi(currentPositionId.id);

        position = poi.GetVertex();
        lastVertexId = poi.GetVertexId();
        roadId = poi.GetNearestEntranceRoadId(position);
    }
    position.z = theAgentGisPtr->GetSubsystem().GetGroundElevationMetersAt(position);
    
    //debug by umeki
    //countAgent1++;
    //std::cout << "initAgentId::" <<  initAgentId << "__AgentId::" << agentId << std::endl;
    //std::cout << "___position.x:" << position.x << "___position.y:" << position.y << std::endl;
    //std::cout << "AgentCount::" << countAgent1 << std::endl;

    for(size_t i = 0; i < NUMBER_TIMESTEP_SNAPSHOTS; i++) {
        status.positions[i] = position;
    }

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters = profilePtr->GetParameters();

    bool hasCar;

    (*this).InitializeStatusWith(parameters, true/*calculateCarAndBicycleProbability*/, hasCar);

    if (hasCar) {
        const Road& road = subsystem.GetRoad(roadId);
        const size_t poiLaneNumber = road.GetParkingLaneNumber();

        vehiclePtr.reset(
            new Vehicle(
                agentId,
                lastVertexId,
                subsystem.GetVertex(lastVertexId),
                roadId,
                poiLaneNumber,
                simulatorPtr));
    }

    taskTablePtr->GetStatusChanges(resource, timeLineStatusChangeEvents);

    mobilityModelPtr.reset(new AgentMobilityModel(resource));

    const bool enableHumanTalkingInterface = false;

    if (enableHumanTalkingInterface) {
        humanInterfacePtr =
            initGlobalNetworkingObjectBag.sensingSubsystemInterfacePtr->CreateShapeSensingModel(
                initSimEngineInterfacePtr,
                agentId,
                "humantalk",
                HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId));

        humanInterfacePtr->SetDataReceiveHandler(
            shared_ptr<SensingModel::DataReceiveHandler>(
                new PhysicalDataReceiver(AgentResource(this))));


        shared_ptr<FanSensingShape> fanSensingShapePtr(new FanSensingShape());

        // assume talking interface for this instance.
        fanSensingShapePtr->SetCoverageDistanceMeters(100/*talking range 10m*/);
        fanSensingShapePtr->SetHorizontalCoverageFromBoresightDegrees(
            135/*-135 <= tallking range <= + 135*/);

        humanInterfacePtr->SetSensingShape(fanSensingShapePtr);
    }

    destinationChangeTrace.UnsetChangeFlag();
}

Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Vehicle>& initVehiclePtr)
    :
    simulatorPtr(initSimulatorPtr),
    theAgentGisPtr(initAgentGisPtr),
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    theRouteSearchSubsystemPtr(initRouteSearchSubsystemPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    currentThreadNumber(0),
    originalThreadNumber(0),
    isDeletedAtTheEndOfTimeStep(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(INFINITE_TIME),
    isTaskInitialized(true),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(0),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    vehicleNodePtr(initVehicleNodePtr),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0)
{
    if (initVehiclePtr != nullptr) {
        currentPositionId = GisPositionIdType(GIS_ROAD, initVehiclePtr->GetRoadId());
        lastVertexId = initVehiclePtr->GetVertexId();

        const Vertex position = theAgentGisPtr->GetVertex(lastVertexId);

        for(size_t i = 0; i < NUMBER_TIMESTEP_SNAPSHOTS; i++) {
            status.positions[i] = position;
        }
    }

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
        profilePtr->GetParameters();

    bool notUsedCarFlag;

    (*this).InitializeStatusWith(parameters, false/*calculateCarAndBicycleProbability*/, notUsedCarFlag);

    AgentResource resource(this);

    mobilityModelPtr.reset(new AgentMobilityModel(resource));

    destinationChangeTrace.UnsetChangeFlag();
}


Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr)
    :
    simulatorPtr(initSimulatorPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    currentThreadNumber(MASTER_THREAD_NUMBER),
    originalThreadNumber(0),
    isDeletedAtTheEndOfTimeStep(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(ZERO_TIME),
    isTaskInitialized(false),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(0),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0)
{
    if (profilePtr != nullptr) {
        AgentResource resource(this);

        const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
            profilePtr->GetParameters();

        bool notUsedCarFlag;

        (*this).InitializeStatusWith(parameters, false/*calculateCarAndBicycleProbability*/, notUsedCarFlag);

        mobilityModelPtr.reset(new AgentMobilityModel(resource));
    }

    destinationChangeTrace.UnsetChangeFlag();
}

shared_ptr<Agent> Agent::CreateMasterAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr)
{
    return shared_ptr<Agent>(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            shared_ptr<SimulationEngineInterface>(),
            initProfilePtr,
            initTaskTablePtr));
}

shared_ptr<Agent> Agent::CreateTaxiDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Taxi>& initTaxiPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr,
            initTaxiPtr));

    agentPtr->currentBehaviorPtr.reset(
        new TaxiDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initTaxiPtr,
            initSimulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));

    return agentPtr;
}

shared_ptr<Agent> Agent::CreateBusDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<Vehicle>& initVehiclePtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Bus>& initBusPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr,
            initVehiclePtr));

    agentPtr->currentBehaviorPtr.reset(
        new BusDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initVehiclePtr,
            initBusPtr,
            initSimulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));

    agentPtr->isTaskInitialized = true;

    return agentPtr;
}

shared_ptr<Agent> Agent::CreateTrainDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Train>& initTrainPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr));

    for(size_t i = 0; i < NUMBER_TIMESTEP_SNAPSHOTS; i++) {
        agentPtr->status.positions[i] = initTrainPtr->GetPosition();
    }

    agentPtr->currentBehaviorPtr.reset(
        new TrainDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initTrainPtr));

    agentPtr->isTaskInitialized = true;

    return agentPtr;
}

Agent::~Agent()
{
    (*this).ReadyToDestruct();
}

void Agent::InitializeStatusWith(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const bool calculateCarAndBicycleProbability,
    bool& hasCar)
{
    AgentResource resource(this);

    status.values.resize(parameters.Size());

    map<string, double> assignedValues;

    const bool isReservedMasterAgent =
        ((agentId == MASTER_ANY_AGENT_ID) ||
         (agentId == MASTER_BUS_AGENT_ID) ||
         (agentId == MASTER_TAXI_AGENT_ID));

    for(AgentStatusIdType statusId = 0; statusId < parameters.Size(); statusId++) {

        if (isReservedMasterAgent) {
            if (!IsBusOrTaxiProfile(statusId)) {
                continue;
            }
        }

        const double value = parameters[statusId].CalculateDouble(resource);

        status.values[statusId] = value;

        assignedValues[parameters.GetLabel(statusId)] = value;
    }

    hasCar = false;
    hasBicycle = false;

    if (calculateCarAndBicycleProbability) {
        HighQualityRandomNumberGenerator& randomNumberGenerator = resource.GetRandomNumberGenerator();

        const double hasCarPercentage = randomNumberGenerator.GenerateRandomDouble();
        const double hasBicyclePercentage = randomNumberGenerator.GenerateRandomDouble();

        hasCar = (profilePtr->GetHasCarRatio(resource) >= hasCarPercentage);
        (*this).hasBicycle = (profilePtr->GetHasBicycleRatio(resource) >= hasBicyclePercentage);
    }

    // Note: Call parent(Simulator) API.
    // Related functions of Simulator must be initialized.
    simulatorPtr->RecordAssignedProfileValuesToFile(
        agentId,
        profilePtr,
        assignedValues,
        hasCar,
        hasBicycle);
}

void Agent::ReadyToDestruct()
{
    // Clear behaviors.
    // Behaviors may refer an agent pointer in an indirect way.
    currentBehaviorPtr.reset();

    typedef set<shared_ptr<AgentCommunicationNode> >::iterator CommNodeIter;

    for(CommNodeIter iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->Detach();
    }

    communicationNodePtrs.clear();

    typedef list<shared_ptr<Agent> >::iterator AgentIter;

    for(AgentIter iter = childAgentPtrs.begin();
        iter != childAgentPtrs.end(); iter++) {

        (*iter)->ReadyToDestruct();
    }

    childAgentPtrs.clear();
}

TimeType Agent::CalculateWakeupTime()
{
    const AgentResource resource(this);

    return taskTablePtr->GetTask(0).GetStartTime(resource);
}

const AgentTask& Agent::CurrentTask() const
{
    if (taskTablePtr->IsEmpty()/*for Taxi*/) {
        assert(!childAgentPtrs.empty());

        if ((currentBehaviorPtr != nullptr) &&
            (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_TAXI_DRIVER)) {

            // Use child(customer) task instead of own task.
            return childAgentPtrs.front()->CurrentTask();
        }
    }

    if (currentIsInterruptTask) {
        return taskTablePtr->GetInterruptTask(currentInterruptTaskNumber);
    }

    return taskTablePtr->GetTask(currentTaskNumber);
}

const GisPositionIdType& Agent::GetHomePositinId() const
{
    if ((currentBehaviorPtr != nullptr) &&
        (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_TAXI_DRIVER)) {

        // Use child(customer) resource instead
        return AgentResource(childAgentPtrs.front().get()).HomePositionId();
    }

    return homePositionId;
}

bool Agent::ShouldChangeRouteInCurrentBehavior(const AgentBehaviorType& specifiedBehavior) const
{
    if (currentBehaviorPtr == nullptr) {
        return false;
    }

    const AgentBehaviorType currentBehaviorType = currentBehaviorPtr->GetBehaviorType();

    if (currentBehaviorPtr->IsInternalRouteChaneMode() &&
        notAvailableBehavorTypesForNextRouteCalculation.find(currentBehaviorType) == notAvailableBehavorTypesForNextRouteCalculation.end() &&
        (specifiedBehavior == AGENT_BEHAVIOR_ANY ||
         currentBehaviorType == specifiedBehavior ||
         currentBehaviorType == AGENT_BEHAVIOR_TAXI_DRIVER ||
         currentBehaviorType == AGENT_BEHAVIOR_BUS_DRIVER)) {
        return true;
    }

    return false;
}
// target now by umeki
// 出発地点から到達地点までのルートを全て選択
void Agent::DecideRoute(const AgentBehaviorType& specifiedBehavior)
{
    AgentResource resource(this);
    

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
        profilePtr->GetParameters();

    lastRouteCalculatedTime = simulatorPtr->CurrentTime();

    status.values[AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME] =
        parameters[AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME].CalculateDouble(resource);

    bool foundRoute = false;
    bool mustNotResetLastsBehavior;

    do {
        const bool isInternalRouteChangeMode = (*this).ShouldChangeRouteInCurrentBehavior(specifiedBehavior);

        bool isDestinationControlledByBehavior = false;
        AgentBehaviorType behaviorToSearchVertex = AGENT_BEHAVIOR_PEDESTRIAN;
        vector<VertexIdType> destVertexIds;
        vector<VertexIdType> startVertexIds;

        if (isInternalRouteChangeMode) {
            assert(currentBehaviorPtr != nullptr);

            behaviorToSearchVertex = currentBehaviorPtr->GetBehaviorType();

            isDestinationControlledByBehavior =
                currentBehaviorPtr->HasFixedDestinationVertex();

            mustNotResetLastsBehavior = currentBehaviorPtr->IsStaticBehavior();
        } else {
            mustNotResetLastsBehavior = false;
        }

        if (isDestinationControlledByBehavior) {

            assert(currentBehaviorPtr != nullptr);
            destVertexIds.push_back(currentBehaviorPtr->GetFixedDestinationVertexId());

        } else {

            if (destPositionId == UNREACHABLE_POSITION_ID) { //目的地が到達不可能な場所ならば

                destVertexId = INVALID_VERTEX_ID;

                // CurrentTask() will return normal or interrupted task due to "currentIsInterruptTask" flag.
                const AgentTask& task = (*this).CurrentTask();

                GisPositionIdType initDestPositionId;
                bool isMultipleDestinations;

                task.GetDestinationId(
                    theAgentGisPtr,
                    !currentIsInterruptTask/*ignoreLastPositionFromCandidate*/,
                    initDestPositionId,
                    isMultipleDestinations,
                    resource);
//                cout << "kokokakakakaak!?!?!!?----\n\n\n"<< endl;

                if (initDestPositionId == UNREACHABLE_POSITION_ID) {
                    (*this).OutputTrace("There is no available destination for current task.");
                    break;
                }

                (*this).SetDestination(
                    initDestPositionId,
                    isMultipleDestinations,
                    false/*byCommunication*/);

                if ((destPositionId == currentPositionId) ||
                    ((extraDestPoiId != UNREACHABLE_POSITION_ID &&
                      extraDestPoiId == currentPositionId))) {
                    break;
                }

                // Assign pass intersections at first time
                if (task.HasPassVertexSpecification()) {
                    task.GetPassVertexIds(
                        theAgentGisPtr,
                        shouldPassVertexIds,
                        resource);
                }
            }

            // return ids sorted by distance from current position.
            theAgentGisPtr->GetNearRouteSearchCandidateVertexIds(
                resource,
                resource.Position(),
                behaviorToSearchVertex,
                destPositionId,
                destVertexId/*prioritizedDestVertexId --> will be inserted to "destVertexIds" as the first entry*/,
                destVertexIds);
        }

        if (currentBehaviorPtr != nullptr) {

            // start next behavior from current behavior via point
            // whether the agent keeps the same behavior type or not.

            startVertexIds.push_back(currentBehaviorPtr->GetViaPointVertexId());

        } else {

            theAgentGisPtr->GetNearRouteSearchCandidateVertexIds(
                resource,
                theAgentGisPtr->GetSubsystem().GetPositionVertex(destPositionId),
                behaviorToSearchVertex,
                currentPositionId,
                lastVertexId,
                startVertexIds);

            if (startVertexIds.empty()) {
                startVertexIds.push_back(lastVertexId);
            }
        }

        assert(!startVertexIds.empty());

        // search from front(prioritized) vertex
        for(size_t i = 0; (!foundRoute && i < destVertexIds.size()); i++) {
            const VertexIdType endVertexId = destVertexIds[i];

            for(size_t j = 0; (!foundRoute && j < startVertexIds.size()); j++) {
                const VertexIdType startVertexId = startVertexIds[j];

                if (isInternalRouteChangeMode) {

                    ostringstream outStream;
                    outStream << "InternalRouteChange v" << startVertexId << " to v" << endVertexId;
                    (*this).OutputTrace(outStream.str());

                    assert(currentBehaviorPtr != nullptr);
                    currentBehaviorPtr->TryInternalRouteChange(startVertexId, endVertexId, foundRoute);

                } else {
                    (*this).DecideRoute(startVertexId, endVertexId, specifiedBehavior, foundRoute);
                }

                if (foundRoute) {
                    if (currentBehaviorPtr == nullptr) {
                        lastVertexId = startVertexId;
                    }
                    destVertexId = endVertexId;
                }
            }
        }

        if (!foundRoute/*--> no route*/ &&
            !isDestinationControlledByBehavior) {

            assert(destPositionId != UNREACHABLE_POSITION_ID);

            unreachableDestinationIds.insert(destPositionId);
            if (extraDestPoiId != UNREACHABLE_POSITION_ID) {
                unreachableDestinationIds.insert(extraDestPoiId);
            }

            (*this).OutputTrace("No Route");

            if (canChangeToOtherDestinationCandidate) {
                (*this).OutputTrace("Destination is unreachable. Search other destination.");
            }

            destPositionId = UNREACHABLE_POSITION_ID;

            if (numberNoRouteStatPtr != nullptr) {
                numberNoRouteStatPtr->IncrementCounter();
            }
        }

    } while (!foundRoute && canChangeToOtherDestinationCandidate);


    if (destPositionId == UNREACHABLE_POSITION_ID) {

        if (mustNotResetLastsBehavior) {

            // nothing to do

        } else {

            currentRouteList.Clear();
            currentBehaviorPtr.reset();
            currentRouteNumber = NO_ROUTE_NUMBER;

            timeToSearchRoute.SetMinTime(resource.CurrentTime());
        }
    }

    (*this).UpdateUtility();
}


// target now by umeki
void Agent::DecideRoute(
    const VertexIdType& startVertexId,
    const VertexIdType endVertexId,
    const AgentBehaviorType& specifiedBehavior,
    bool& foundRoute)
{
    foundRoute = false;

    const AgentResource resource(this);

    shared_ptr<AgentRoute> lastRoutePtr;

    if (currentBehaviorPtr != nullptr) {
        lastRoutePtr = currentRouteList.routePtrs.front();
    }

    vector<list<AgentRouteList> > routeCandidatesPerOrder;

    typedef list<AgentRouteList>::const_iterator RouteIter;

    vector<pair<RouteIter, double> > routeIters;
    double totalRouteWeight = 0;
    bool hasBeaviorRoute = false;

    if (destPositionId != currentPositionId &&
        startVertexId != endVertexId) {
        const AgentTask& task = (*this).CurrentTask();

        ostringstream outStream;
        outStream << "SearchRoute v" << startVertexId << " to v" << endVertexId;
        (*this).OutputTrace(outStream.str());

        const AgentBehaviorType preferedBehavior = task.GetPreferedBehavior();
        AgentBehaviorType behavior = task.GetBehavior();
        if (specifiedBehavior != AGENT_BEHAVIOR_ANY) {
            // overwrite behavior
            behavior = specifiedBehavior;
        }

        theRouteSearchSubsystemPtr->SearchRouteCandidates(
            resource,
            startVertexId,
            endVertexId,
            shouldPassVertexIds,
            timeToSearchRoute,
            preferedBehavior,
            notAvailableBehavorTypesForNextRouteCalculation,
            routeCandidatesPerOrder);

        assert(!routeCandidatesPerOrder.empty());

        if (behavior != AGENT_BEHAVIOR_ANY) {
            for(size_t i = 0; (!hasBeaviorRoute && i < routeCandidatesPerOrder.size()); i++) {

                const list<AgentRouteList>& routeCandidates = routeCandidatesPerOrder[i];

                for(RouteIter iter = routeCandidates.begin();
                    (!hasBeaviorRoute && iter != routeCandidates.end()); iter++) {

                    if ((*iter).totalCost.values[AGENT_ROUTE_COST_MODE] > 0) {
                        hasBeaviorRoute = true;
                    }
                }
            }
        }
    }

    for(size_t i = 0; i < routeCandidatesPerOrder.size(); i++) {
        const list<AgentRouteList>& routeCandidates = routeCandidatesPerOrder[i];

        for(RouteIter iter = routeCandidates.begin(); iter != routeCandidates.end(); iter++) {
            const AgentRouteList& aRoute = (*iter);

            if (aRoute.IsEmpty()) {
                continue;
            }
            if (hasBeaviorRoute && aRoute.totalCost.values[AGENT_ROUTE_COST_MODE] <= 0) {
                continue;
            }

            const double routeWeight = (*this).CalculateRouteWeight(aRoute);

            routeIters.push_back(make_pair(iter, routeWeight));
            totalRouteWeight += routeWeight;

            (*this).OutputTrace(
                "RouteCandidate " + ConvertToString(routeWeight) + ":" +
                aRoute.ConvertToString());
        }
    }

    if ((!routeIters.empty()) ||
        (startVertexId == endVertexId && destPositionId != currentPositionId) ||
        (destPositionId == currentPositionId &&
         extraDestPoiId != extraCurrentPoiId &&
         extraDestPoiId.IsValid())) {

        foundRoute = true;
        currentRouteNumber++;

        currentRouteList.Clear();
        timeToSearchRoute.SetMinTime(resource.CurrentTime());

        shared_ptr<AgentRoute> nextRoutePtr;

        if (!routeIters.empty()) {
            size_t routeId = 0;
            HighQualityRandomNumberGenerator& randomnumberGenerator = resource.GetRandomNumberGenerator();

            
            if (totalRouteWeight > 0) {
                const double decideRatio = randomnumberGenerator.GenerateRandomDouble();

                double totalRatio = 0;

                for(; routeId < routeIters.size() - 1; routeId++) {
                    totalRatio += (routeIters[routeId].second / totalRouteWeight);

                    if (decideRatio < totalRatio) {
                        break;
                    }
                }
            } else {
                routeId = randomnumberGenerator.GenerateRandomInt(0, (int32_t)(routeIters.size() - 1));
            }
            currentRouteList = *(routeIters[routeId].first);

            assert(!currentRouteList.routePtrs.empty());

            if (currentBehaviorPtr != nullptr &&

                currentBehaviorPtr->IsAcceptableRouteChange(*currentRouteList.routePtrs.front())) {
                currentBehaviorPtr->ChangeRoute(currentRouteList.routePtrs.front());

            } else {

                if (currentBehaviorPtr != nullptr) {

                    if (!currentRouteList.routePtrs.empty()) {
                        nextRoutePtr = currentRouteList.routePtrs.front();
                    }

                    currentRouteList.routePtrs.push_front(lastRoutePtr);
                    currentBehaviorPtr->EndBehaviorAtViaPoint(nextRoutePtr);

                } else {
                    if (!currentRouteList.IsEmpty()) {
                        if (IsFreeWalkPosition(currentPositionId.type)) {
                            const TimeType preparetionTime = 30*SECOND;

                            currentRouteList.routePtrs.push_front(
                                shared_ptr<AgentRoute>(new AgentRoute(AGENT_BEHAVIOR_FREEWALK)));

                            currentRouteList.startTime -= preparetionTime;
                        }
                    }

                    currentTaskStartTime =
                        std::max(currentTaskStartTime, currentRouteList.startTime);
                }
            }
        } else {
            if (currentBehaviorPtr != nullptr) {
                currentRouteList.routePtrs.push_front(lastRoutePtr);
                currentBehaviorPtr->EndBehaviorAtViaPoint(nextRoutePtr);
            }
        }

        if (IsFreeWalkPosition(destPositionId.type)) {
            currentRouteList.routePtrs.push_back(
                shared_ptr<AgentRoute>(new AgentRoute(AGENT_BEHAVIOR_FREEWALK)));
        }

        (*this).OutputTrace(
            "Route:" + currentRouteList.ConvertToString());
    }
}

double Agent::CalculateRouteWeight(const AgentRouteList& aRoute)
{
    const AgentResource resource(this);

    const double totalWeight =
        profilePtr->CalculateRouteUtility(
            resource, AGENT_ROUTE_COST_MODE, healthOrUtilityFactor, aRoute.totalCost);

    return std::max(0., totalWeight);
}

void Agent::SetDestination(
    const GisPositionIdType& initDestPositionId,
    const bool initCanChangeToOtherDestinationCandidate,
    const bool byCommunication)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    GisPositionIdType newDestPositionId = initDestPositionId;
    GisPositionIdType newExtraDestPoiId = UNREACHABLE_POSITION_ID;

    if (newDestPositionId.type == GIS_POI) {
        const Poi& poi = subsystem.GetPoi(newDestPositionId.id);

        if (poi.IsAPartOfObject()) {
            newDestPositionId = poi.GetParentGisPositionId();

            assert(newDestPositionId.type == GIS_BUILDING ||
                   newDestPositionId.type == GIS_PARK);

            newExtraDestPoiId = initDestPositionId;
        }
    }

    if (destPositionId != newDestPositionId &&
        newDestPositionId != UNREACHABLE_POSITION_ID) {
        destinationChangeTrace.SetValue(*destinationChangeTrace + 1);

        if (byCommunication) {
            destinationChangeByCommunicationTrace.SetValue(*destinationChangeByCommunicationTrace + 1);
        }

        const string name = subsystem.GetGisObject(initDestPositionId).GetObjectName();

        ostringstream outStream;
        outStream << "SetDestination " << name;

        (*this).OutputTrace(outStream.str());
    }

    shouldPassVertexIds.clear();
    destPositionId = newDestPositionId;
    extraDestPoiId = newExtraDestPoiId;

    canChangeToOtherDestinationCandidate = initCanChangeToOtherDestinationCandidate;
    recalculateRoute = true;
}

void Agent::SetCurrentDestinationToUnreachablePosition()
{
    AgentResource resource(this);

    resource.UnreachableDestinationNotification();

    list<GisPositionIdType> unreachablePositionIds;

    unreachablePositionIds.push_back(destPositionId);

    if (extraDestPoiId != UNREACHABLE_POSITION_ID) {
        unreachablePositionIds.push_back(extraDestPoiId);
    }

    (*this).AddUnreachablePositions(unreachablePositionIds, false/*byCommunication*/);
}

void Agent::AddUnreachablePositions(
    const list<GisPositionIdType>& unreachablePositionIds,
    const bool byCommunication)
{
    AgentResource resource(this);

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    typedef list<GisPositionIdType>::const_iterator IterType;

    for(IterType iter = unreachablePositionIds.begin(); iter != unreachablePositionIds.end(); iter++) {
        const GisPositionIdType& positionId = (*iter);

        // agent is in a position
        if (positionId == currentPositionId && !resource.WaitingAtEntrance()) {
            continue;
        }

        unreachableDestinationIds.insert(positionId);
    }

    if (unreachableDestinationIds.find(destPositionId) != unreachableDestinationIds.end()) {

        if ((*this).WaitingAtDestinationEntrace()) {
            theAgentGisPtr->PeopleGiveUpEntrance(resource, destPositionId);
        }

        if (byCommunication) {
            destinationChangeByCommunicationTrace.SetValue(*destinationChangeByCommunicationTrace + 1);
        }
        destPositionId = UNREACHABLE_POSITION_ID;
        recalculateRoute = true;
    }
}

void Agent::ChangeToSpecificDestination(
    const GisPositionIdType& initDestPositionId,
    const VertexIdType& initDestVertexId,
    const bool byCommunication)
{
    AgentResource resource(this);

    if ((*this).WaitingAtDestinationEntrace()) {
        theAgentGisPtr->PeopleGiveUpEntrance(resource, currentPositionId);
    }

    (*this).SetDestination(initDestPositionId, false/*isMultipleDestinations*/, byCommunication);

    const TimeType currentTime = simulatorPtr->CurrentTime();

    currentTaskStartTime = currentTime;
    destVertexId = initDestVertexId;

    (*this).RecalculateRoute(currentTime);
}

void Agent::IncrementTime(const size_t threadNumber)
{
    currentThreadNumber = threadNumber;

    if (isDeletedAtTheEndOfTimeStep) {  //シミュレーション終了条件
        return;
        cout << "finish??" << endl;
    }
        
   
    // initialization (called only once at first time)
    if (!isTaskInitialized) {
        isTaskInitialized = true;
        (*this).AssignCurrentTask();
    }

    AgentResource resource(this);


    
 
    
    // status(profile variable or task) changes
    (*this).ApplyStatusChangesAndInstantiateApplications();

    // route (re)calculation
    if (resource.ExceededRouteRecalculationTime()) {
        resource.RecalculateRoute();
    }
    if (recalculateRoute) {
        (*this).DecideRoute(recalculateRouteWithBehavior);　//ルート決定

        recalculateRoute = false;
        notAvailableBehavorTypesForNextRouteCalculation.clear();

        if (numberRouteCalculateTimeStatPtr != nullptr) {
            numberRouteCalculateTimeStatPtr->IncrementCounter();
        }

        recalculateRoute = false;
        recalculateRouteWithBehavior = AGENT_BEHAVIOR_ANY;
    }

    const TimeType currentTime = simulatorPtr->CurrentTime();
    const GisPositionIdType lastPositionId = currentPositionId;
    const AgentBehaviorType lastBehaviorType = resource.GetBehaviorType();
    const int currentAgentId = resource.AgentId();  //add by Ubi

    // execute behavior
    if (currentTaskStartTime <= currentTime) {
        bool assignedNewBehavior;

        (*this).RecalculateBehaviorIfNecessary(assignedNewBehavior);

        if (assignedNewBehavior) {
            //Increment from next time step.
            resource.SetPosition(resource.Position());
        } else {
            (*this).IncrementCurrentBehaviorTime();
        }

    } else {
        resource.SetPosition(resource.Position());
    }

    // next behavior preparation
    if ((*this).CurrentTaskHasFinished()) {
        (*this).OutputTrace("Finished Task");

        (*this).GoToNextBehaviorIfPossible();

    // Give up current destination and go to other destination.
    } else if ((*this).OtherDestinationSeemsToBeBetter()) {

        assert(resource.WaitingAtEntrance());
        assert(resource.ExceededWaitEntranceTime());

        (*this).OutputTrace("Destination reached a limit capacity. Set other destination");

        (*this).SetCurrentDestinationToUnreachablePosition();
    }

    // update translation state
    theAgentGisPtr->UpdatePeopleTranslationBetweenGisObjects(
        resource,
        lastPositionId,
        currentPositionId,
        lastBehaviorType,
        resource.GetBehaviorType());

    // statistics/traces
    const Vertex& currentPos = (*this).GetCurrentPosition();
    const Vertex& nextPos = (*this).GetNextPosition();

    if (utility1StatPtr != nullptr) {
        utility1StatPtr->RecordStatValue(resource.Utility1());
    }
    if (utility2StatPtr != nullptr) {
        utility2StatPtr->RecordStatValue(resource.Utility2());
    }

    const double distanceMeters = currentPos.DistanceTo(nextPos);
    status.values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_DISTANCE] += currentPos.DistanceTo(nextPos);
    
    if(moveFlag == true && escapeAgentInfoTemp[currentAgentId - minAgentId].totalTravelDistance == resource.TotalTravelDistance()){
        escapeAgentInfoTemp[currentAgentId - minAgentId].arrivedFlagCount++;
    } else{
        escapeAgentInfoTemp[currentAgentId - minAgentId].arrivedFlagCount = 0;
    }
    
    if(escapeAgentInfoTemp[currentAgentId - minAgentId].arrivedFlagCount == 100) {    //移動を数秒間停止することを目的地に到着すると仮定する
        if(!escapeAgentInfoTemp[currentAgentId - minAgentId].getArrivedFlag()){
            escapeAgentInfoTemp[currentAgentId - minAgentId].setArrivedFlag(true);
            agentArrivedCount++;
            assert(currentAgentId == escapeAgentInfoTemp[currentAgentId - minAgentId].getObjectAgentId());
            if(agentArrivedCount%1000 == 0){
                cout << "end the Escape AgentId::" << currentAgentId << "\tescapedCount::" << agentArrivedCount << endl;
            }

            if(agentAddCount == agentArrivedCount){
                std::ofstream("./result.csv");
                std::ofstream("./result2_congestion.csv");
                cout << "Agent Escape End" << "\tEnd Time::" << resource.TotalTravelTime() << endl;
                fstream writing_file, writing_file_congestion;
                writing_file.open("./result.csv", std::ios::out);   //scenargie directory:[visuallab/scemtemp~~/sim] in visuallab
                writing_file_congestion.open("./result2_congestion.csv", std::ios::out);

                // 被災者情報リストCSVの作成
                for(int i=0; i < agentAddCount; i++){
                    if(i == 0){
                        writing_file << "AgentId,Congestion,TotalTravelTime,TotalTravelGain,TurnOffCount,shelterPrefix,";
                        for(int j=0; j < escapeAgentInfoTemp[i].getShelterNumber(); j++)
                            writing_file << "shelterGainNo" << j << ",";
                        writing_file << endl;
                    }
                    cout << escapeAgentInfoTemp[i].getObjectAgentId() << "I want to get agentTotalDistance::"
                            << escapeAgentInfoTemp[i].totalTravelGain << endl;
                    
                    writing_file << escapeAgentInfoTemp[i].getObjectAgentId() << ","    //AgentId
                                << escapeAgentInfoTemp[i].congestion << ","             //Congestion
                                << escapeAgentInfoTemp[i].totalTravelTime << ","        //TotalTravelTime
                                << escapeAgentInfoTemp[i].totalTravelTime * -1.0 << "," //TotalTravelGain
                                << escapeAgentInfoTemp[i].getTurnOffCount() << ","      //TurnOff Count(たらい回し)
                                << escapeAgentInfoTemp[i].getShelterPrefix() << ",";    //ObjectiveShelterId
                    for(int j=0; j < escapeAgentInfoTemp[i].getShelterNumber(); j++){   //each shelterGain
                        writing_file << escapeAgentInfoTemp[i].getAgentGain(j) << ",";
                    }
                    writing_file << endl;
                }

                // 時系列混雑リストCSVの作成
                for(int i=0; i < MAX_SIM; i++){
                    if(i == 0){
                        writing_file_congestion << "Time(every10seconds), congestion" << endl;
                    }
                    writing_file_congestion << i*10 << "," << congestionTimeList[i] << endl;
                    if(1000 < i && congestionTimeList[i] == 0) break;
                }
                exit(1);
            }
        }
    }
    
    if (distanceMeters >= MinStepDistanceToCountInStatsMeters) {

        status.values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_TIME] +=
            double(simulatorPtr->TimeStep()) / SECOND;

        if (travelDistanceStatPtr != nullptr) {
            const double traveDistance = resource.TotalTravelDistance();
            travelDistanceStatPtr->RecordStatValue(traveDistance);
            travelDistanceTrace.SetValue(traveDistance);
        }
        if (travelTimeStatPtr != nullptr) {
            const double travelTime = resource.TotalTravelTime();
            travelTimeStatPtr->RecordStatValue(travelTime);
            travelTimeTrace.SetValue(travelTime);
        }
        assert(currentAgentId == escapeAgentInfoTemp[currentAgentId - minAgentId].getObjectAgentId());
        escapeAgentInfoTemp[currentAgentId - minAgentId].congestion += resource.Congestion();
        //cout << "congestion::" << resource.Congestion() << endl;
        if((resource.CurrentTime() % 10000000000) == 0){
            congestionTimeList[resource.CurrentTime() / 10000000000] += resource.Congestion();
        }
        escapeAgentInfoTemp[currentAgentId - minAgentId].totalTravelTime = resource.TotalTravelTime();
        escapeAgentInfoTemp[currentAgentId - minAgentId].totalTravelGain = -1.0 * resource.TotalTravelTime();
        escapeAgentInfoTemp[currentAgentId - minAgentId].totalTravelDistance = resource.TotalTravelDistance();

        ifstream ifs("./result.csv");
        if(!ifs.is_open()){
        }
        assert(agentAddCount > 0);
        escapeAgentInfoTemp[currentAgentId - minAgentId].ovrideAgentGain(
                escapeAgentInfoTemp[currentAgentId - minAgentId].getShelterPrefix(),
                -1.0 * resource.TotalTravelTime());
        //ここで落ちる場合は、おそらくBuildingの位置情報を取得できていないため、避難先が決定していない可能性がある
        moveFlag = true;
    }


    if(agentRouteSearchCount + 1 == agentAddCount && moveFlag == true){ //シミュ時間10秒ごとに出力
        spacedSecondCount++;
        if(spacedSecondCount == 5000){
            cout << "Travel Time:" << resource.CurrentTime() << "\tNow Escaped Agent::" << agentArrivedCount << endl;
        }
    }
    
    if(moveFlag){
        agentRouteSearchCount++;
        if(agentRouteSearchCount == agentAddCount) agentRouteSearchCount = 0;
        if(spacedSecondCount == 5000) spacedSecondCount = 0;
    }
    
    // chldren calculation
    typedef list<shared_ptr<Agent> >::const_iterator IterType;

    for(IterType iter = childAgentPtrs.begin(); iter != childAgentPtrs.end(); iter++) {
        assert(agentId != (*iter)->GetAgentId());
        (*iter)->IncrementTime(threadNumber);
    }
}

void Agent::GoToNextBehaviorIfPossible()
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    AgentResource resource(this);

    if (currentIsInterruptTask) {

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                currentTime,
                currentInterruptTaskNumber,
                AGENT_STATUS_CHANGE_TASK_INTERRUPTION_END));

        if ((*this).CurrentTask().GetInterruptionType() == AGENT_BEHAVIOR_INTERRUPTION_LATER) {
            // back to last (interrupted) normal task.
            if (currentTaskNumber > 0) {
                currentTaskNumber--;
            }
        }

        currentIsInterruptTask = false;

    } else {
        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                currentTime,
                currentTaskNumber,
                AGENT_STATUS_CHANGE_BASIC_TASK_END));
    }


    do {
        currentTaskNumber++;

        if ((*this).HasCurrentTask()) {
            if ((*this).CurrentTask().SatisfyCondition(resource)) {
                (*this).AssignCurrentTask();
                break;
            }
        }

    } while ((*this).HasCurrentTask());

    if (!(*this).HasCurrentTask()) {
        (*this).OutputTrace("Completed All Task");
        isDeletedAtTheEndOfTimeStep = true;
        currentPositionId = GisPositionIdType();
    }
}

void Agent::AssignCurrentTask()
{
    assert(!currentIsInterruptTask);

    const AgentTask& task = (*this).CurrentTask();

    AgentResource resource(this);

    currentTaskEndTime = task.GetEndTime(resource);
    currentTaskStartTime = task.GetStartTime(resource);

    const TimeType earlyStartTime =
        std::max(resource.CurrentTime(), currentTaskStartTime);

    task.GetTimeLine(resource, earlyStartTime, timeToSearchRoute);

    timeLineStatusChangeEvents.push(
        AgentStatusChangeEvent(
            earlyStartTime,
            currentTaskNumber,
            AGENT_STATUS_CHANGE_BASIC_TASK_START));

    destPositionId = UNREACHABLE_POSITION_ID;

    (*this).RecalculateRoute(simulatorPtr->CurrentTime());
}

void Agent::AssignInterruptTask()
{
    assert(currentIsInterruptTask);

    AgentResource resource(this);

    if ((*this).WaitingAtDestinationEntrace()) {
        theAgentGisPtr->PeopleGiveUpEntrance(resource, currentPositionId);
    }

    const AgentTask& task = (*this).CurrentTask();

    currentTaskEndTime = task.GetEndTime(resource);
    currentTaskStartTime = task.GetStartTime(resource);

    assert(currentTaskStartTime == simulatorPtr->CurrentTime());

    task.GetTimeLine(resource, currentTaskStartTime, timeToSearchRoute);

    destPositionId = UNREACHABLE_POSITION_ID;

    (*this).RecalculateRoute(simulatorPtr->CurrentTime());
}

void Agent::ApplyStatusChangesAndInstantiateApplications()
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    while (!timeLineStatusChangeEvents.empty() &&
           timeLineStatusChangeEvents.top().time <= currentTime) {

        const AgentStatusChangeEvent& statusChangeEvent = timeLineStatusChangeEvents.top();
        const AgentResource resource(this);
        const AgentStatusChangeType& changeType = statusChangeEvent.statusChangeType;

        if (IsBasicTaskStatusChange(changeType)) {

            if (statusChangeEvent.statusChangeNumber < taskTablePtr->GetNumberOfTasks()) {
                const AgentTask& task = taskTablePtr->GetTask(statusChangeEvent.statusChangeNumber);

                if (task.HasStatusChange(changeType)) {
                    (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                }
            }

        } else if (IsInterrupionTaskStatusChange(changeType)) {

            if (statusChangeEvent.statusChangeNumber < taskTablePtr->GetNumberOfInterruptTasks()) {
                const AgentTask& task = taskTablePtr->GetInterruptTask(statusChangeEvent.statusChangeNumber);

                if (changeType == AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START) {

                    if (task.SatisfyCondition(resource)) {
                        (*this).OutputTrace("InterruptCurrentAction");

                        currentIsInterruptTask = true;
                        currentInterruptTaskNumber = statusChangeEvent.statusChangeNumber;

                        if (task.HasStatusChange(changeType)) {
                            (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                        }

                        (*this).AssignInterruptTask();
                    }
                } else {

                    if (task.HasStatusChange(changeType)) {
                        (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                    }
                }
            }

        } else {
            assert(IsSpecificTimeStatusChange(changeType));

            const AgentTask& task = taskTablePtr->GetStatusChange(statusChangeEvent.statusChangeNumber);

            if (task.SatisfyCondition(resource)) {
                (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
            }
        }

        timeLineStatusChangeEvents.pop();
    }
}

void Agent::ApplyAdditionalStatusChanges(const AgentAdditionalStatusChange& additionalStatusChange)
{
    (*this).ApplyStatusChanges(additionalStatusChange.statusChanges);
    (*this).InstantiateApplications(additionalStatusChange.applicationSpecifications);
}

void Agent::ApplyStatusChanges(const vector<pair<AgentStatusIdType, AgentValueFormula> >& statusChanges)
{
    if (statusChanges.empty()) {
        return;
    }

    const AgentResource resource(this);

    bool needToUpdateVehicleStatus = false;

    ostringstream outStream;
    outStream << "ChangeStatus: ";

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters = profilePtr->GetParameters();

    for(size_t i = 0; i < statusChanges.size(); i++) {
        const pair<AgentStatusIdType, AgentValueFormula>& statusChange = statusChanges[i];
        const AgentStatusIdType& statusId = statusChange.first;
        const AgentValueFormula& valueFormula = statusChange.second;

        status.values[statusId] = valueFormula.CalculateDouble(resource);

        if (i > 0) {
            outStream << ", ";
        }
        outStream << parameters.GetLabel(statusId) << " = " << status.values[statusId];

        if (IsVehicleStatus(statusId)) {
            needToUpdateVehicleStatus = true;
        }
    }

    (*this).OutputTrace(outStream.str());

    if (needToUpdateVehicleStatus && vehiclePtr != nullptr) {
        theAgentGisPtr->UpdateVehicleStatus(resource, vehiclePtr);
    }
}

void Agent::InstantiateApplications(
    const map<DynamicApplicationIdType, DynamicApplicationDefinition>& applicationSpecifications)
{
    if (applicationSpecifications.empty()) {
        return;
    }

    typedef map<DynamicApplicationIdType, DynamicApplicationDefinition>::const_iterator IterType;

    const AgentResource resource(this);
    const NodeIdType nodeId = agentId;
    const TimeType currentTime = simulatorPtr->CurrentTime();

    for(IterType iter = applicationSpecifications.begin();
        iter != applicationSpecifications.end(); iter++) {

        const DynamicApplicationIdType& dynamicApplicationId = (*iter).first;
        const DynamicApplicationDefinition& applicationDefinition = (*iter).second;
        const vector<DynamicApplicationDefinitionParameter>& parameters = applicationDefinition.parameters;
        const string& instanceName = dynamicApplicationId.instanceName;

        InterfaceOrInstanceIdType instanceId;

        if (!instanceName.empty()) {
            instanceId = instanceName + "-" + ConvertToString(nodeId);
        } else {
            instanceId = "dynamicapp" + ConvertTimeToStringSecs(currentTime) + "s-" + ConvertToString(nodeId);
        }

        ostringstream scopeStream;

        scopeStream << "[" << nodeId << ";" << instanceId << "]";

        const string scope = scopeStream.str();

        vector<string> parameterLines;
        set<NodeIdType> targetNodeIds;

        targetNodeIds.insert(nodeId);

        for(size_t i = 0; i < parameters.size(); i++) {
            const DynamicApplicationDefinitionParameter& parameter = parameters[i];

            if (parameter.applicationParameterName.find("-destination") != string::npos) {
                const NodeIdType destinationNodeId = App_ConvertStringToNodeIdOrAnyNodeId(parameter.value);

                targetNodeIds.insert(destinationNodeId);
            }

            const string& parameterName = parameter.applicationParameterName;
            string parameterNameWithSpace = parameter.applicationParameterName + " ";
            string value = parameter.value;

            // Search with white space end indicator to avoid finding "-start-time-xxx" and "-end-time-xxx".

            if ((parameterNameWithSpace.find("-start-time ") != string::npos) ||
                (parameterNameWithSpace.find("-end-time ") != string::npos)) {

                TimeType timeValue;
                bool success;

                ConvertStringToTime(value, timeValue, success);

                if (!success) {
                    cerr << "Error: Application Behavior, bad Time parameter value for: " << parameterName;
                    cerr << "  Value = " << value << endl;
                    exit(1);
                }

                timeValue += currentTime;

                value = ConvertTimeToStringSecs(timeValue);
            }

            const string parameterLine = scope + " " + parameterName + " = " + value;

            parameterLines.push_back(parameterLine);

        }

        ostringstream outStream;
        outStream << "GenerateApplication: ";

        for(size_t i = 0; i < parameterLines.size(); i++) {
            if (i > 0) {
                outStream << ", ";
            }
            outStream << parameterLines[i];
        }

        (*this).OutputTrace(outStream.str());

        simulatorPtr->CreateApplicationForNode(
            resource,
            nodeId,
            instanceId,
            parameterLines,
            targetNodeIds);
    }
}

void Agent::RecalculateBehaviorIfNecessary(bool& assignedNewBehavior)
{
    AgentResource resource(this);

    if (currentBehaviorPtr != nullptr || resource.WaitingAtEntrance()) {
        assignedNewBehavior = false;
        return;
    }

    if (currentRouteList.IsEmpty()) {
        assignedNewBehavior = false;
        return;
    }

    assignedNewBehavior = true;

    (*this).OutputTrace("Assign Behavior for " + currentRouteList.ConvertToString());

    const shared_ptr<AgentRoute> routePtr = currentRouteList.routePtrs.front();
    const TimeType currentTime = resource.CurrentTime();

    currentBehaviorStartTime = currentTime;
    currentBehaviorCost = AgentRouteCost();

    const shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr = simulatorPtr->GetPublicVehicleTable();

    switch (routePtr->behavior) {
    case AGENT_BEHAVIOR_FREEWALK: {

        GisPositionIdType endPositionId = destPositionId;
        bool enterToPosition;

        if (currentRouteList.routePtrs.size() > 1) {
            enterToPosition = false;// -> leave from position

            list<shared_ptr<AgentRoute> >::const_iterator iter =
                currentRouteList.routePtrs.begin();

            iter++;

            if ((*iter)->IsRoad()) {
                endPositionId = (*iter)->GetStartRoadPositionId();
            } else {
                endPositionId = thePublicVehicleTablePtr->GetPositionId((*iter)->GetStartStopId());
            }

        } else {
            enterToPosition = true;

            assert(endPositionId.type == GIS_BUILDING ||
                   endPositionId.type == GIS_PARK ||
                   endPositionId.type == GIS_POI);

            resource.SetPositionId(endPositionId);
        }

        currentBehaviorPtr.reset(
            new FreeWalkBehavior(theAgentGisPtr, thePublicVehicleTablePtr, routePtr, endPositionId, enterToPosition, resource));
        break;
    }

    case AGENT_BEHAVIOR_VEHICLE:
        currentBehaviorPtr.reset(
            new VehicleDriverBehavior(
                theAgentGisPtr, thePublicVehicleTablePtr, routePtr, resource, vehiclePtr, simulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));
        break;

    case AGENT_BEHAVIOR_BUS:
    case AGENT_BEHAVIOR_TRAIN:
        currentBehaviorPtr.reset(
            new PublicVehicleBehavior(
                theAgentGisPtr, thePublicVehicleTablePtr, routePtr, resource));
        break;

    case AGENT_BEHAVIOR_TAXI:
        currentBehaviorPtr.reset(
            new TaxiGuestBehavior(theAgentGisPtr, thePublicVehicleTablePtr, routePtr, resource));
        break;

    case AGENT_BEHAVIOR_BICYCLE:
        currentBehaviorPtr.reset(
            new BicycleBehavior(theAgentGisPtr, thePublicVehicleTablePtr, routePtr, resource));
        break;

    default:
        currentBehaviorPtr.reset(
            new PedestrianBehavior(theAgentGisPtr, thePublicVehicleTablePtr, routePtr, resource));
        break;
    }

    if (currentBehaviorPtr != nullptr) {
        (*this).OutputTrace("New Behavior:" + currentBehaviorPtr->GetBehaviorName());
    }

    (*this).UpdateUtility();
}

void Agent::IncrementCurrentBehaviorTime()
{
    AgentResource resource(this);

    if (currentBehaviorPtr == nullptr || resource.WaitingAtEntrance()) {
        resource.SetPosition(resource.Position());
        return;
    }

    currentBehaviorPtr->IncrementTimeStep(simulatorPtr->TimeStep());

    if (MultiAgentSimulator::isDebugMode) {
        (*this).OutputTrace(currentBehaviorPtr->MakePositionTraceString());
    }//if//

    if (!recalculateRoute && currentBehaviorPtr->HasFinished()) {

        lastDelay = resource.NextDelay();
        (*this).UpdateUtility();

        currentBehaviorPtr.reset();

        (*this).OutputTrace("End Behavior");

        if (!currentRouteList.IsEmpty()) {

            currentRouteList.routePtrs.pop_front();
        }

        if (currentRouteList.routePtrs.empty() && (*this).HasCurrentTask()) {
            const TimeType currentTime = resource.CurrentTime();
            const TimeType waitTime = (*this).CurrentTask().GetWaitTime(resource);
            const TimeType waitEndTime = currentTime + waitTime;

            if (waitTime > ZERO_TIME) {
                if (currentIsInterruptTask) {
                    timeLineStatusChangeEvents.push(
                        AgentStatusChangeEvent(
                            currentTime,
                            currentInterruptTaskNumber,
                            AGENT_STATUS_CHANGE_TASK_INTERRUPTION_BEFORE_WAITING));
                } else {
                    timeLineStatusChangeEvents.push(
                        AgentStatusChangeEvent(
                            currentTime,
                            currentTaskNumber,
                            AGENT_STATUS_CHANGE_BASIC_TASK_BEFORE_WAITING));
                }
            }

            if (currentTaskEndTime < INFINITE_TIME) {
                currentTaskEndTime = std::max(currentTaskEndTime, waitEndTime);
            } else {
                currentTaskEndTime = waitEndTime;
            }
        }
    }

    //(*this).UpdateHealthFactor();
}

void Agent::UpdateHealthFactor()
{
    AgentResource resource(this);

    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_LAST_DELAY] = double(resource.LastDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_NEXT_DELAY] = double(resource.NextDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_TRIP_DELAY] = double(resource.TripDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_ARRIVAL_DELAY] = double(resource.ArrivalDelay()) / SECOND;
}

void Agent::UpdateUtility()
{
    (*this).UpdateHealthFactor();

    const AgentResource resource(this);

    utility1CalculationCount++;
    utility2CalculationCount++;

    status.values[AGENT_RESERVED_STATUS_UTILITY1] =
        profilePtr->CalculateUtility1(resource, healthOrUtilityFactor, currentRouteList.totalCost);
    status.values[AGENT_RESERVED_STATUS_UTILITY2] =
        profilePtr->CalculateUtility2(resource, healthOrUtilityFactor, currentRouteList.totalCost);
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_UTILITY1_COUNTER] = utility1CalculationCount;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_UTILITY2_COUNTER] = utility2CalculationCount;

    const double utility1 = resource.Utility1();
    const double utility2 = resource.Utility2();

    utility1Trace.SetValue(utility1);
    utility2Trace.SetValue(utility2);

    (*this).OutputTrace("Utility = " + ConvertToString(utility1) + " / " +  ConvertToString(utility2));
}

bool Agent::CurrentTaskHasFinished() const
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    if (currentTime < currentTaskStartTime) {
        return false;
    }

    if (currentBehaviorPtr != nullptr) {
        return false;
    }

    if (!currentRouteList.IsEmpty()) {
        return false;
    }

    // 1. Dynamically generated agents in gis object which has capacity may wait entrance.
    // 2. Agents are going to skip current task if there is no route and try to leave(;be deleted) from simulation at the end of the last task.
    // Guarantee the agent entrance before the agent deletion.

    if ((*this).WaitingAtEntrance()) {
        return false;
    }

    if (currentTaskEndTime < INFINITE_TIME) {
        return (currentTime >= currentTaskEndTime);
    }

    return true;
}

bool Agent::HasCurrentTask() const
{
    return (currentTaskNumber < taskTablePtr->GetNumberOfTasks());
}

void Agent::AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    communicationNodePtr->resource = AgentResource(this); //connect resource

    communicationNodePtrs.insert(communicationNodePtr);

    communicationNodePtr->Attach(mobilityModelPtr);
}

void Agent::DeleteCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    assert(communicationNodePtrs.find(communicationNodePtr) != communicationNodePtrs.end());

    communicationNodePtrs.erase(communicationNodePtr);

    communicationNodePtr->Detach();
}

void Agent::RecalculateRoute(
    const TimeType& recalculateStartTime,
    const AgentBehaviorType& initRecalculateRouteWithBehavior)
{
    timeToSearchRoute.SetMinTime(recalculateStartTime);

    recalculateRoute = true;
    recalculateRouteWithBehavior = initRecalculateRouteWithBehavior;
}

void Agent::SetVertexId(const VertexIdType& vertexId)
{
    if (lastVertexId != vertexId) {

        if (!shouldPassVertexIds.empty()) {
            if (shouldPassVertexIds.front() == vertexId) {
                shouldPassVertexIds.pop_front();
            }
        }

        if (!communicationNodePtrs.empty()) {
            const Vertex position = theAgentGisPtr->GetVertex(vertexId);

            typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

            for(IterType iter = communicationNodePtrs.begin();
                iter != communicationNodePtrs.end(); iter++) {
                (*iter)->ArrivedAtVertexNotification(vertexId, position);
            }
        }
    }

    lastVertexId = vertexId;
}

bool Agent::WaitingAtDestinationEntrace() const
{
    return (currentPositionId == destPositionId && (*this).WaitingAtEntrance());
}

bool Agent::WaitingAtEntrance() const
{
    return (entranceWaitStartTime != INFINITE_TIME);
}

bool Agent::OtherDestinationSeemsToBeBetter()
{
    if (!canChangeToOtherDestinationCandidate) {
        return false;
    }

    if ((*this).WaitingAtDestinationEntrace()) {
        const AgentResource resource(this);

        if (resource.ExceededWaitEntranceTime()) {

            return true;
        }
    }

    return false;
}

const Vertex& Agent::GetCurrentPosition() const
{
    return status.positions[simulatorPtr->GetCurrentSnapshotId()];
}

const Vertex& Agent::GetNextPosition() const
{
    return status.positions[simulatorPtr->GetNextSnapshotId()];
}

AgentProfileType AgentResource::ProfileType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetProfileType();
}

TimeType AgentResource::CurrentTime() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->simulatorPtr->CurrentTime();
}

TimeType AgentResource::CurrentBehaviorSpentTime() const
{
    (*this).CheckAgentAvailability();

    return ((*this).CurrentTime() - agentPtr->currentBehaviorStartTime);
}

AgentMobilityClassType AgentResource::MobilityClass() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetMobilityClass();
}

AgentTicketType AgentResource::TicketType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetTicketType();
}

AgentUserType AgentResource::UserType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetUserType();
}

const Vertex& AgentResource::Position() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetCurrentPosition();
}

string AgentResource::GetProfileName() const
{
    if (agentPtr == nullptr) {
        return string();
    }

    return agentPtr->profilePtr->GetProfileName();
}

const Vertex& AgentResource::DebugNextPosition() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetNextPosition();
}

ObjectMobilityPosition AgentResource::MobilityPosition() const
{
    (*this).CheckAgentAvailability();

    const Vertex& position = (*this).Position();
    const TimeType currentTime = (*this).CurrentTime();

    return ObjectMobilityPosition(
        currentTime,
        currentTime,
        position.x,
        position.y,
        position.z,
        true/*theHeightContainsGroundHeightMeters*/,
        agentPtr->agentId,
        90 - agentPtr->directionRadians*(180./PI));
}

ObjectMobilityPosition AgentResource::MobilityPositionForTime(const TimeType& time) const
{
    (*this).CheckAgentAvailability();

    const Vertex& position = (*this).Position();
    const TimeType currentTime = (*this).CurrentTime();

    return ObjectMobilityPosition(
        currentTime,
        currentTime,
        position.x,
        position.y,
        position.z,
        true/*theHeightContainsGroundHeightMeters*/,
        agentPtr->agentId,
        90 - agentPtr->directionRadians*(180./PI));
}

AgentBehaviorType AgentResource::GetBehaviorType() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return AGENT_BEHAVIOR_NOTHING;
    }

    return agentPtr->currentBehaviorPtr->GetBehaviorType();
}

bool AgentResource::WaitingAtEntrance() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->WaitingAtEntrance();
}

bool AgentResource::ExceededWaitEntranceTime() const
{
    assert((*this).WaitingAtEntrance());
    (*this).CheckAgentAvailability();

    return ((*this).CurrentTime() >= agentPtr->entranceWaitStartTime + (*this).EntranceWaitTime());
}

const GisPositionIdType& AgentResource::DestPositionId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->destPositionId;
}

const GisPositionIdType& AgentResource::ExtraDestPoiId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->extraDestPoiId;
}

const GisPositionIdType& AgentResource::HomePositionId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetHomePositinId();
}

RouteNumberType  AgentResource::CurrentRouteNumber() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->currentRouteNumber;
}

const AgentRoute& AgentResource::CurrentRoute() const
{
    (*this).CheckAgentAvailability();

    assert(!agentPtr->currentRouteList.routePtrs.empty());
    return *agentPtr->currentRouteList.routePtrs.front();
}

StopIdType AgentResource::GetNextRouteStopId() const
{
    (*this).CheckAgentAvailability();
    assert(agentPtr->currentRouteList.routePtrs.size() > 1);

    typedef list<shared_ptr<AgentRoute> >::const_iterator IterType;

    IterType iter = agentPtr->currentRouteList.routePtrs.begin();
    iter++;

    return (*iter)->GetStartStopId();
}

const set<GisPositionIdType>& AgentResource::UnreachableDestinationIds() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->unreachableDestinationIds;
}

const AgentTaskPurposeType& AgentResource::CurrentPurpose() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->CurrentTask().GetPurpose();
}

HighQualityRandomNumberGenerator& AgentResource::GetRandomNumberGenerator() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->aRandomNumberGenerator;
}

HighQualityRandomNumberGenerator& AgentResource::GetRandomNumberGeneratorForDestinationChoice() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->aRandomNumberGeneratorForDestinationChoice;
}

TimeType AgentResource::LastDelay() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->lastDelay;
}

TimeType AgentResource::NextDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }
    if (agentPtr->currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_FREEWALK) {
        return ZERO_TIME;
    }

    const AgentRouteCost& routeCost = agentPtr->currentBehaviorPtr->GetRoute().totalCost;
    const TimeType currentTime = (*this).CurrentTime();

    return std::max(ZERO_TIME, currentTime - routeCost.ArrivalTime());
}

TimeType AgentResource::TripDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }

    if (!agentPtr->timeToSearchRoute.specifiedArrivalTime) {
        return ZERO_TIME;
    }

    const AgentRouteCost& routeCost = agentPtr->currentRouteList.totalCost;

    return std::max(ZERO_TIME, routeCost.ArrivalTime() - agentPtr->timeToSearchRoute.arrivalTime);
}

TimeType AgentResource::ArrivalDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }

    if (!agentPtr->timeToSearchRoute.specifiedArrivalTime) {
        return ZERO_TIME;
    }

    const TimeType currentTime = (*this).CurrentTime();

    return std::max(ZERO_TIME, currentTime - agentPtr->timeToSearchRoute.arrivalTime);
}

void AgentResource::SetPosition(const Vertex& position)
{
    (*this).CheckAgentAvailability();

    (*this).Status().positions[agentPtr->simulatorPtr->GetNextSnapshotId()] = position;
}

void AgentResource::SetVertexId(const VertexIdType& vertexId)
{
    (*this).CheckAgentAvailability();

    agentPtr->SetVertexId(vertexId);
}

void AgentResource::SetCongestion(const double value)
{
    (*this).CheckAgentAvailability();

    agentPtr->healthOrUtilityFactor.values[AGENT_HEALTH_FACTOR_CONGESTION] = value;

    agentPtr->congestionTrace.SetValue((*this).Congestion());
}

bool AgentResource::ExceededRouteRecalculationTime() const
{
    (*this).CheckAgentAvailability();

    const TimeType recalculationTime = (*this).RouteRecalculationTime();

    if (recalculationTime <= ZERO_TIME) {
        return false;
    }

    return ((*this).CurrentTime() > agentPtr->lastRouteCalculatedTime + recalculationTime);
}

void AgentResource::SetLastPathQueryTriggerTime()
{
    (*this).CheckAgentAvailability();

    agentPtr->lastPathQueryTriggerTime = (*this).CurrentTime();
}

bool AgentResource::IsPathQueryTriggerAvailable(
    const double congestion,
    const TimeType& vehicleDelay) const
{
    (*this).CheckAgentAvailability();

    const double pathQueryProbability = (*this).PathQueryProbability();

    if (pathQueryProbability <= 0) {
        return false;
    }

    const TimeType minPathQueryDuration = (*this).MinPathQueryInterval();

    if ((*this).CurrentTime() <= agentPtr->lastPathQueryTriggerTime + minPathQueryDuration) {
        return false;
    }

    HighQualityRandomNumberGenerator& randomnumberGenerator = (*this).GetRandomNumberGenerator();

    const double aValue = randomnumberGenerator.GenerateRandomDouble();

    if (aValue > pathQueryProbability) {
        return false;
    }

    const bool isAvailable =
        ((*this).LastDelay() >= (*this).LastDelayQueryTrigger()*SECOND ||
         (*this).NextDelay() >= (*this).NextDelayQueryTrigger()*SECOND ||
         (*this).TripDelay() >= (*this).TripDelayQueryTrigger()*SECOND ||
         vehicleDelay >= (*this).VehicleDelayQueryTrigger()*SECOND ||
         congestion >= (*this).CongestionQueryTrigger() ||
         (*this).Utility1() >= (*this).Utility1QueryTrigger() ||
         (*this).Utility2() >= (*this).Utility2QueryTrigger());

    ostringstream outStream;
    outStream << "PathQuery = " << isAvailable
              << ", LastDelay = " << (*this).LastDelay() / SECOND
              << ", NextDelay = " << (*this).NextDelay() / SECOND
              << ", TripDelay = " << (*this).TripDelay() / SECOND
              << ", ArrivalDelay = " << (*this).ArrivalDelay() / SECOND
              << ", VehicleDelay = " << vehicleDelay / SECOND
              << ", Congestion = " << congestion
              << ", Utility = " << (*this).Utility1() << " / " << (*this).Utility2();

    (*this).OutputTrace(outStream.str());

    return isAvailable;
}

void AgentResource::AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr)
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr != nullptr) {
        agentPtr->currentBehaviorPtr->AssignTaxi(initTaxiPtr);
    }
}

void AgentResource::RecalculateRoute(const TimeType& recalculateStartTime)
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute(recalculateStartTime);
}

void AgentResource::RecalculateRoute()
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute((*this).CurrentTime());
}

void AgentResource::RecalculateRouteWithBehavior(const AgentBehaviorType& behavior)
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute((*this).CurrentTime(), behavior);
}

void AgentResource::RecalculateRouteWithNotAvailableBehaviorSpecification(
    const set<AgentBehaviorType>& notAvailableBehaviorTypes)
{
    (*this).CheckAgentAvailability();

    agentPtr->notAvailableBehavorTypesForNextRouteCalculation = notAvailableBehaviorTypes;
    agentPtr->RecalculateRoute((*this).CurrentTime());
}

void AgentResource::ArrivedAtDeadEndNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtDeadEndNotification();
    }
}

void AgentResource::ArrivedAtDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtDestinationNotification();
    }
}

void AgentResource::EnteredToDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->EnteredToDestinationNotification();
    }
}

void AgentResource::ArrivedAtGisPositionNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;
    const GisPositionIdType currentPositionId = agentPtr->currentPositionId;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtGisPositionNotification(currentPositionId);
    }
}

void AgentResource::ReceivePhysicalData(SensingSharedInfoType& broadcastData)
{
    assert((*this).IsAvailable());

    // receiving process
}

void AgentResource::UnreachableDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;
    const GisPositionIdType& destPositionId = agentPtr->destPositionId;

    if (destPositionId == UNREACHABLE_POSITION_ID) {
        return;
    }

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->UnreachableDestinationNotification(destPositionId);
    }
}

void AgentResource::SetDestination(const Vertex& position, const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    const GisSubsystem& subsystem = agentPtr->theAgentGisPtr->GetSubsystem();

    vector<GisObjectType> prioritizedDestinationObjectTypes;
    prioritizedDestinationObjectTypes.push_back(GIS_POI);
    prioritizedDestinationObjectTypes.push_back(GIS_BUILDING);
    prioritizedDestinationObjectTypes.push_back(GIS_PARK);

    const GisPositionIdType positionId =
        subsystem.GetPositionId(position, prioritizedDestinationObjectTypes);

    if (positionId.IsInvalid()) {
        return;
    }

    (*this).SetDestination(positionId, position, byCommunication);
}

void AgentResource::SetDestination(
    const GisPositionIdType& positionId,
    const Vertex& position,
    const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    const GisSubsystem& subsystem = agentPtr->theAgentGisPtr->GetSubsystem();
    const VertexIdType destVertexId = subsystem.GetNearestVertexId(positionId, position);

    agentPtr->ChangeToSpecificDestination(positionId, destVertexId, byCommunication);
}

void AgentResource::AddUnreachablePositions(
    const list<GisPositionIdType>& unreachablePositionIds,
    const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    agentPtr->AddUnreachablePositions(unreachablePositionIds, byCommunication);
}

void AgentResource::SetPositionId(const GisPositionIdType& positionId)
{
    (*this).CheckAgentAvailability();

    agentPtr->currentPositionId = positionId;
}

void AgentResource::SetExtraPoiId(const GisPositionIdType& positionId)
{
    (*this).CheckAgentAvailability();

    agentPtr->extraCurrentPoiId = positionId;
}

void AgentResource::SetPositionId(const GisObjectType& objectType, const VariantIdType& variantId)
{
    (*this).CheckAgentAvailability();

    (*this).SetPositionId(GisPositionIdType(objectType, variantId));
}

void AgentResource::SetDirectionRadians(const double directionRadians)
{
    (*this).CheckAgentAvailability();

    agentPtr->directionRadians = directionRadians;
}

void AgentResource::SetOwnerAgent(
    const AgentIdType& ownerAgentId)
{
    (*this).CheckAgentAvailability();

    agentPtr->simulatorPtr->SetOwnerAgent(*this, ownerAgentId);
}

void AgentResource::RemoveOwnerAgent()
{
    (*this).CheckAgentAvailability();

    if (agentPtr->HasParent()) {
        (*this).SetOwnerAgent();
    } else {
        agentPtr->simulatorPtr->RemoveOwnerAgentChange(*this);
    }
}

void AgentResource::WaitEntrance()
{
    if ((*this).WaitingAtEntrance()) {
        return;
    }

    (*this).CheckAgentAvailability();

    agentPtr->entranceWaitStartTime = (*this).CurrentTime();
}

void AgentResource::AllowedEntrance()
{
    if (!(*this).WaitingAtEntrance()) {
        return;
    }

    (*this).CheckAgentAvailability();

    agentPtr->entranceWaitStartTime = INFINITE_TIME;

    if ((*this).PositionId() == (*this).DestPositionId()) {
        (*this).EnteredToDestinationNotification();
    }
}

//----------------------------------------------------------------------------------

enum AgentParameterValueType {
    AGENT_PARAMETER_VALUE_INT,
    AGENT_PARAMETER_VALUE_DOUBLE,
    AGENT_PARAMETER_VALUE_STRING,
};

class NoFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values.front();
    }
};
class PlusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values[0] + values[1];
    }
};
class MinusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values[0] - values[1];
    }
};
class DivFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        if (values[1] == 0) {
            cerr << "Error: occured 0 division. Check agent profile forumula." << endl;
            exit(1);
        }
        return values[0] / values[1];
    }
};
class MultiFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values[0] * values[1];
    }
};
class ModFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        if (values[1] == 0) {
            cerr << "Error: occured 0 mod. Check agent profile forumula." << endl;
            exit(1);
        }
        return int(values[0]) % int(values[1]);
    }
};
class EplusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values[0] * std::pow(10, values[1]);
    }
};
class EminusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return values[0] * std::pow(10, -values[1]);
    }
};
class Log10Formula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {

        return log10(values[0]);
    }
};
class LogNFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return log(values[0]);
    }
};
class PowFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return pow(values[0], values[1]);
    }
};
class MinFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return std::min(values[0], values[1]);
    }
};
class MaxFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return std::max(values[0], values[1]);
    }
};
class SqrtFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (sqrt(values[0]));
    }
};
class SinFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (std::sin(values[0]));
    }
};
class CosFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (std::cos(values[0]));
    }
};
class TanFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (std::tan(values[0]));
    }
};
class AbsFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return fabs(values[0]);
    }
};
class CeilFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return ceil(values[0]);
    }
};
class FloorFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return floor(values[0]);
    }
};
class PiFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return PI;
    }
};
class ExpFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (exp(values[0]));
    }
};
class UniFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return double(resource.GetRandomNumberGenerator().GenerateRandomInt(
                          static_cast<int>(values[0]),
                          static_cast<int>(values[1])));
    }
};
class UnidFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        const double largeValue = std::max(values[0], values[1]);
        const double samllValue = std::min(values[0], values[1]);

        if (largeValue == samllValue) {
            return samllValue;
        }

        return ((largeValue - samllValue) * resource.GetRandomNumberGenerator().GenerateRandomDouble() + samllValue);
    }
};
class NormalFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {

        HighQualityRandomNumberGenerator& aRandomNumberGenerator = resource.GetRandomNumberGenerator();

        // central limit theorem

        double total = 0;

        for (int i = 0; i < 12; i++) {
            total += aRandomNumberGenerator.GenerateRandomDouble();
        }

        return (values[1] * (total - 6.0) + values[0]);
    }
};
class ExpDistributionFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {
        return (- values[0] * log(1 -  resource.GetRandomNumberGenerator().GenerateRandomDouble()));
    }
};
class PoissonFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {

        HighQualityRandomNumberGenerator& aRandomNumberGenerator = resource.GetRandomNumberGenerator();

        int i = 0;
        double aValue = exp(values[0])*aRandomNumberGenerator.GenerateRandomDouble();

        for(; aValue > 1.0; i++) {
            aValue *= aRandomNumberGenerator.GenerateRandomDouble();
        }

        return static_cast<double>(i);
    }
};
class ErlangFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource) const {

        HighQualityRandomNumberGenerator& aRandomNumberGenerator = resource.GetRandomNumberGenerator();

        const int phase = static_cast<int>(values[1]);
        double aValue = 1.0;

        for(int i = 0; i < phase; i++) {
            aValue *= (1 - aRandomNumberGenerator.GenerateRandomDouble());
        }

        return (- values[0] / static_cast<double>(phase) * log(aValue));
    }
};

const shared_ptr<AgentValueFormula::Formula> AgentValueFormula::formulaPtrs[] = {
    shared_ptr<AgentValueFormula::Formula>(new NoFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PlusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MinusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new DivFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MultiFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ModFormula()),
    shared_ptr<AgentValueFormula::Formula>(new EplusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new EminusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new Log10Formula()),
    shared_ptr<AgentValueFormula::Formula>(new LogNFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PowFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MinFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MaxFormula()),
    shared_ptr<AgentValueFormula::Formula>(new SqrtFormula()),
    shared_ptr<AgentValueFormula::Formula>(new SinFormula()),
    shared_ptr<AgentValueFormula::Formula>(new CosFormula()),
    shared_ptr<AgentValueFormula::Formula>(new TanFormula()),
    shared_ptr<AgentValueFormula::Formula>(new AbsFormula()),
    shared_ptr<AgentValueFormula::Formula>(new CeilFormula()),
    shared_ptr<AgentValueFormula::Formula>(new FloorFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PiFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ExpFormula()),
    shared_ptr<AgentValueFormula::Formula>(new UniFormula()),
    shared_ptr<AgentValueFormula::Formula>(new UnidFormula()),
    shared_ptr<AgentValueFormula::Formula>(new NormalFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ExpDistributionFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PoissonFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ErlangFormula()),
};

static inline
bool CanRemoveOutmostArc(const string& aString)
{
    if (aString.empty() ||
        !(aString[0] == '(' && aString[aString.length() - 1] == ')')) {
        return false;
    }

    size_t numberStartArcs = 1;

    for(size_t i = 1; i < aString.size() - 1; i++) {

        if (aString[i] == '(') {
            numberStartArcs++;
        } else if (aString[i] == ')') {
            numberStartArcs--;

            if (numberStartArcs == 0) {
                return false;
            }
        }
    }

    return (numberStartArcs == 1);
}

static inline
bool IsFunctionString(const string& aString)
{
    if (aString.empty() || !isalpha(aString[0])) {
        return false;
    }

    size_t functionNameEndPos = aString.find_first_of("+-*/%^(.");

    if (functionNameEndPos != string::npos &&
        aString[functionNameEndPos] == '(') {

        const string functionContents = aString.substr(functionNameEndPos);

        return CanRemoveOutmostArc(functionContents);
    }

    return false;
}

static inline
bool IsVariableString(const string& aString)
{
    if (aString.empty() ||
        !(isalpha(aString[0]) || aString[0] == '_'))  {
        return false;
    }

    return (aString.find_first_of("+-*/%^(.") == string::npos);
}

static inline
void ConvertTimeStringToDoubleSec(
    const string& aString, double& value, bool& success)
{
    deque<string> timeStrings;
    TokenizeToTrimmedLowerStringWithArc(aString, ":", timeStrings);

    int hour = 0;
    int minute = 0;
    int second = 0;

    ConvertStringToInt(timeStrings[0], hour, success);
    if (!success) {
        return;
    }

    ConvertStringToInt(timeStrings[1], minute, success);
    if (!success) {
        return;
    }

    if (timeStrings.size() >= 3) {
        ConvertStringToInt(timeStrings[2], second, success);
        if (!success) {
            return;
        }
    }

    success = true;
    value = 60*60*hour + 60*minute + second;
}

static inline
bool IsEnumValue(const string& aString)
{
    if (IsFunctionString(aString)) {
        return false;
    }

    bool success;
    double aValue;

    ConvertStringToDouble(aString, aValue, success);

    return !success;
}

static inline
void TokenizeToConditionOrTaskString(
    const string& aString,
    vector<string>& tokens)
{
    tokens.clear();

    size_t lastToneStartPos = 0;
    size_t currentPos = 0;

    while (currentPos < aString.size()) {
        const size_t arcPos = aString.find('(', lastToneStartPos);

        size_t numberArcs = 0;

        for(currentPos = arcPos; currentPos < aString.size(); currentPos++) {
            const char aChar = aString[currentPos];

            if (aChar == '(') {
                numberArcs++;
            } else if (aChar == ')') {
                numberArcs--;
            }

            if (numberArcs == 0) {
                tokens.push_back(aString.substr(lastToneStartPos, currentPos - lastToneStartPos + 1));
                lastToneStartPos = currentPos + 1;
                break;
            }
        }
    }
}

static inline
string NonspacedString(const string& aString)
{
    string nonspacedString;

    for(size_t i = 0; i < aString.size(); i++) {
        if (aString[i] != ' ') {
            nonspacedString.push_back(aString[i]);
        }
    }

    return nonspacedString;
}

static inline
string NonspacedRawFormulaString(const string& aString)
{
    string nonspacedString = NonspacedString(aString);

    size_t numberStartArcs = 0;
    size_t numberEndArcs = 0;

    for(size_t i = 0; i < nonspacedString.size(); i++) {
        const char aChar = nonspacedString[i];

        if (aChar == '(') {
            numberStartArcs++;
        } else  if (aChar == ')') {
            numberEndArcs++;
        }
    }

    if (numberStartArcs != numberEndArcs) {
        cerr << "Error: lack of arcs in formula " << aString << endl;
        exit(1);
    }

    while (CanRemoveOutmostArc(nonspacedString)) {

        nonspacedString = NonspacedString(nonspacedString.substr(1, nonspacedString.length() - 2));
    }

    return nonspacedString;
}

AgentValueFormula::AgentValueFormula(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds,
    const double initDefaultValue)
    :
    inputFormulaString(aString),
    defaultValue(initDefaultValue)
{
    if (!aString.empty()) {

        string nonEqualString = NonspacedRawFormulaString(aString);

        if (nonEqualString[0] == '=') {
            nonEqualString = nonEqualString.substr(1);
        }

        if (nonEqualString != "-") {
            (*this).AddFormulaUnitRecursively(
                parameters, simStartTimeSec, nonEqualString, charactorIds);

            assert(formulaUnits.size() > 0);
            (*this).PrecalculateFormulaUnit(0);
        }
    }
}

AgentValueFormula::AgentValueFormula(
    const string& aString,
    const double initDefaultValue)
    :
    inputFormulaString(aString),
    defaultValue(initDefaultValue)
{
    LabelMap<AgentStatusIdType, AgentValueFormula> noParameters;
    const double simStartTimeSec = 0;
    map<string, AgentCharactorIdType> noCharactorIds;

    if (!aString.empty()) {

        string nonEqualString = NonspacedRawFormulaString(aString);

        if (nonEqualString[0] == '=') {
            nonEqualString = nonEqualString.substr(1);
        }

        if (nonEqualString != "-") {
            (*this).AddFormulaUnitRecursively(
                noParameters, simStartTimeSec, nonEqualString, noCharactorIds);

            assert(formulaUnits.size() > 0);
            (*this).PrecalculateFormulaUnit(0);
        }
    }
}

void AgentValueFormula::PrecalculateFormulaUnit(const FormulaUnitIdType& unitId)
{
    const AgentResource emptyResource(nullptr);

    FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);

    if (formulaUnit.operation != FORMULA_OPERATION_NONE) {
        vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

        for(size_t i = 0; i < values.size(); i++) {
            pair<FormulaUnitIdType, double>& aValue = values[i];
            const FormulaUnitIdType valueUnitId = aValue.first;

            if (valueUnitId != NO_FORMULA_UNIT_ID) {
                assert(valueUnitId != unitId);

                const FormulaUnit& valueFormulaUnit =
                    (*this).GetFormulaUnit(valueUnitId);

                if (MayBeConstValue(valueFormulaUnit.operation)) {
                    (*this).PrecalculateFormulaUnit(valueUnitId);
                }
                if (valueFormulaUnit.CompletedAllCalculation()) {
                    aValue.second = (*this).Calculate(valueUnitId, emptyResource);
                    aValue.first = NO_FORMULA_UNIT_ID;
                }
            }
        }
    }
}

AgentValueFormula::FormulaUnit& AgentValueFormula::GetFormulaUnit(const FormulaUnitIdType& unitId)
{
    assert(unitId != NO_FORMULA_UNIT_ID);
    assert(unitId < formulaUnits.size());
    return formulaUnits[unitId];
}

const AgentValueFormula::FormulaUnit& AgentValueFormula::GetFormulaUnit(const FormulaUnitIdType& unitId) const
{
    assert(unitId != NO_FORMULA_UNIT_ID);
    assert(unitId < formulaUnits.size());
    return formulaUnits[unitId];
}

bool AgentValueFormula::FormulaUnit::CompletedAllCalculation() const
{
    if (operation >= FORMULA_OPERATION_DISTRIBUTION_START) {
        return false;
    }

    for(size_t i = 0; i < values.size(); i++) {
        if (values[i].first != NO_FORMULA_UNIT_ID) {
            return false;
        }
    }

    return true;
}

TimeType AgentValueFormula::CalculateTime(
    const AgentResource& resource,
    const bool calculateMaxValue) const
{
    if ((*this).IsNull()) {
        if (calculateMaxValue) {
            return INFINITE_TIME;
        }
        return resource.CurrentTime();
    }

    return static_cast<TimeType>((*this).CalculateDouble(resource, calculateMaxValue) * SECOND);
}

double AgentValueFormula::CalculateDouble(
    const AgentResource& resource,
    const bool calculateMaxValue) const
{
    if ((*this).IsNull()) {
        return defaultValue;
    }

    return (*this).Calculate(0, resource, calculateMaxValue);
}

double AgentValueFormula::CalculateUtility(
    const AgentResource& resource,
    const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
    const AgentRouteCost& cost) const
{
    if ((*this).IsNull()) {
        return defaultValue;
    }

    return (*this).CalculateUtility(0, resource, healthOrUtilityFactor, cost);
}

double AgentValueFormula::Calculate(
    const FormulaUnitIdType& unitId,
    const AgentResource& resource,
    const bool calculateMaxValue) const
{
    assert(unitId < formulaUnits.size());
    assert(unitId != NO_FORMULA_UNIT_ID);

    const FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);
    const vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

    vector<double> doubleValues(values.size());

    for(size_t i = 0; i < values.size(); i++) {
        const pair<FormulaUnitIdType, double>& value = values[i];
        if (value.first == NO_FORMULA_UNIT_ID) {
            doubleValues[i] = value.second;
        } else {
            assert(unitId != value.first);
            doubleValues[i] = (*this).Calculate(value.first, resource, calculateMaxValue);
        }
    }

    if (IsAgentStatus(formulaUnit.operation)) {
        return resource.Value(ConvertTofAgentStatusId(formulaUnit.operation));
    }

    if (calculateMaxValue) {
        if (formulaUnit.operation == FORMULA_OPERATION_UNI ||
            formulaUnit.operation == FORMULA_OPERATION_UNID) {
            return std::max(doubleValues[0], doubleValues[1]);
        } else if (formulaUnit.operation == FORMULA_OPERATION_NORMAL) {
            return doubleValues[1] * 6.0 + doubleValues[0];
        } else if (formulaUnit.operation == FORMULA_OPERATION_EXP_DISTRIBUTION ||
                   formulaUnit.operation == FORMULA_OPERATION_POISSON ||
                   formulaUnit.operation == FORMULA_OPERATION_ERLANG) {
            return DBL_MAX;
        }
    }

    if (formulaUnit.operation >= NUMBER_FORMULA_TYPES) {
        cerr << "Error: Predefined reserved parameter is available only in UtilityFunction/RoutePriority" << endl;
        exit(1);
    }

    return (*formulaPtrs[formulaUnit.operation])(doubleValues, resource);
}

double AgentValueFormula::CalculateUtility(
    const FormulaUnitIdType& unitId,
    const AgentResource& resource,
    const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
    const AgentRouteCost& cost) const
{
    assert(unitId < formulaUnits.size());
    assert(unitId != NO_FORMULA_UNIT_ID);

    const FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);
    const vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

    vector<double> doubleValues(values.size());

    for(size_t i = 0; i < values.size(); i++) {
        const pair<FormulaUnitIdType, double>& value = values[i];
        if (value.first == NO_FORMULA_UNIT_ID) {
            doubleValues[i] = value.second;
        } else {
            assert(unitId != value.first);
            doubleValues[i] = (*this).CalculateUtility(value.first, resource, healthOrUtilityFactor, cost);
        }
    }

    if (IsRouteCost(formulaUnit.operation)) {
        return cost.values[ConvertToRouteCostId(formulaUnit.operation)];
    }

    if (IsHealthOrUtilityCost(formulaUnit.operation)) {
        return healthOrUtilityFactor.values[
            ConvertToHealthOrUtilityFactorId(formulaUnit.operation)];
    }

    if (IsAgentStatus(formulaUnit.operation)) {
        return resource.Value(ConvertTofAgentStatusId(formulaUnit.operation));
    }

    assert(formulaUnit.operation < NUMBER_FORMULA_TYPES);
    return (*formulaPtrs[formulaUnit.operation])(doubleValues, resource);
}

void AgentValueFormula::ResolveOperation(
    const string& aString,
    string& leftValue,
    string& rightValue,
    FormulaOperationType& operation)
{
    assert(!aString.empty());

    const string nonspacedString = NonspacedRawFormulaString(aString); //remove outmost "()"

    assert(!IsFunctionString(nonspacedString));
    int currentPos;

    int numberRemainingFirstArcs = 0;

    bool found = false;
    for(currentPos = (int)(nonspacedString.size() - 1); currentPos >= 0; currentPos--) {
        const char aChar = nonspacedString[currentPos];

        if (aChar == '(') {
            numberRemainingFirstArcs--;
        } else  if (aChar == ')') {
            numberRemainingFirstArcs++;
        }

        if (numberRemainingFirstArcs == 0 &&
            ((aChar == '+')  || (aChar == '-'))) {

            if (currentPos == 0 ||
                (currentPos > 0 && nonspacedString[currentPos - 1] != 'e')) {
                found  = true;
            }
            break;
        }
    }

    if (!found) {
        for(currentPos = (int)(nonspacedString.size() - 1); currentPos >= 0; currentPos--) {
            const char aChar = nonspacedString[currentPos];

            if (aChar == '(') {
                numberRemainingFirstArcs--;
            } else  if (aChar == ')') {
                numberRemainingFirstArcs++;
            }

            if (numberRemainingFirstArcs == 0 &&
                ((aChar == '/') || (aChar == '*') || (aChar == '%') || (aChar == '^') || (aChar == '+') || (aChar == '-'))) {
                break;
            }
        }
    }

    if (currentPos < 0) {//>= int(nonspacedString.size())) {
        leftValue = aString;
        operation = FORMULA_OPERATION_NONE;
        return;
    }

    leftValue = nonspacedString.substr(0, currentPos);

    assert(currentPos < int(nonspacedString.size()));

    const char aChar = nonspacedString[currentPos];

    if (aChar == '+') {

        if (currentPos > 0 && nonspacedString[currentPos - 1] == 'e') {

            operation = FORMULA_OPERATION_E_PLUS;
            leftValue = nonspacedString.substr(0, currentPos - 1);

        } else {
            operation = FORMULA_OPERATION_PLUS;
        }

    } else if (aChar == '-') {

        if (currentPos > 0 && nonspacedString[currentPos - 1] == 'e') {

            operation = FORMULA_OPERATION_E_MINUS;
            leftValue = nonspacedString.substr(0, currentPos - 1);

        } else {
            operation = FORMULA_OPERATION_MINUS;
        }

    } else if (aChar == '/') {
        operation = FORMULA_OPERATION_DIV;
    } else if (aChar == '*') {
        operation = FORMULA_OPERATION_MULTI;
    } else if (aChar == '%') {
        operation = FORMULA_OPERATION_MOD;
    } else if (aChar == '^') {
        operation = FORMULA_OPERATION_POW;
    } else {
        cerr << "Error: invalid operation '" << aChar << "' at " << aString << endl;
        exit(1);
    }

    rightValue = nonspacedString.substr(currentPos + 1);
}

AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddFormulaUnitRecursively(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds)
{
    const string nonspacedString = NonspacedRawFormulaString(aString);

    if (IsVariableString(nonspacedString)) {

        return (*this).AddVariableUnit(parameters, nonspacedString);

    } else if (IsFunctionString(nonspacedString)) {

        return (*this).AddFunctionUnitRecursively(
            parameters, simStartTimeSec, nonspacedString, charactorIds);

    } else {

        const FormulaUnitIdType unitId = formulaUnits.size();
        formulaUnits.push_back(FormulaUnit(unitId));

        string leftValue;
        string rightValue;

        FormulaOperationType operation;

        (*this).ResolveOperation(nonspacedString, leftValue, rightValue, operation);

        (*this).GetFormulaUnit(unitId).operation = operation;

        assert(operation < NUMBER_FORMULA_TYPES);

        if (operation == FORMULA_OPERATION_NONE) {
            bool success;
            double aValue;

            if (aString.find(":") != string::npos) {
                double timeSec;
                ConvertTimeStringToDoubleSec(aString, timeSec, success);

                if (timeSec < simStartTimeSec) {
                    cerr << "Error: invalid time" << aString << endl;
                    exit(1);
                }
                aValue = timeSec - simStartTimeSec;
            } else {
                ConvertStringToDouble(aString, aValue, success);
            }

            // Static cast workaround on Linux for Microsoft violation of C++ standard.

            (*this).GetFormulaUnit(unitId).values.push_back(
                make_pair(static_cast<FormulaUnitIdType>(NO_FORMULA_UNIT_ID), aValue));
        } else {
            FormulaUnitIdType leftUnitId = NO_FORMULA_UNIT_ID;
            double noValue = 0;

            if (!leftValue.empty()) {
                leftUnitId =
                    (*this).AddFormulaUnitRecursively(
                        parameters, simStartTimeSec, leftValue, charactorIds);
            }

            const FormulaUnitIdType rightUnitId =
                (*this).AddFormulaUnitRecursively(
                    parameters, simStartTimeSec, rightValue, charactorIds);

            assert(unitId != leftUnitId);
            assert(unitId != rightUnitId);

            (*this).GetFormulaUnit(unitId).values.push_back(make_pair(leftUnitId, noValue));
            (*this).GetFormulaUnit(unitId).values.push_back(make_pair(rightUnitId, noValue));
        }

        return unitId;
    }
}

AgentValueFormula::FormulaOperationType AgentValueFormula::GetFormulaOperation(
    const string& functionString,
    const size_t numberArguments) const
{
    if (numberArguments < 1) {
        cerr << "Error: Function definition needs one or more arguments" << endl;
        exit(1);
    }

    if (functionString == "log10") {
        return FORMULA_OPERATION_LOG10;
    } else if (functionString == "logn" ||
               functionString == "log" ||
               functionString == "logarithm") {
        return FORMULA_OPERATION_LOGN;
    } else if (functionString == "pow" || functionString == "power") {
        return FORMULA_OPERATION_POW;
    } else if (functionString == "min" || functionString == "minimum") {
        return FORMULA_OPERATION_MIN;
    } else if (functionString == "max" || functionString == "maximum") {
        return FORMULA_OPERATION_MAX;
    } else if (functionString == "sqrt" || functionString == "squareroot") {
        return FORMULA_OPERATION_SQRT;
    } else if (functionString == "sin" || functionString == "sine") {
        return FORMULA_OPERATION_SIN;
    } else if (functionString == "cos" || functionString == "cosine") {
        return FORMULA_OPERATION_COS;
    } else if (functionString == "tan" || functionString == "tangent") {
        return FORMULA_OPERATION_TAN;
    } else if (functionString == "abs" || functionString == "absolute") {
        return FORMULA_OPERATION_ABS;
    } else if (functionString == "ceil" || functionString == "ceiling") {
        return FORMULA_OPERATION_CEIL;
    } else if (functionString == "floor") {
        return FORMULA_OPERATION_FLOOR;
    } else if (functionString == "pi") {
        return FORMULA_OPERATION_PI;
    } else if (functionString == "exp" || functionString == "exponential") {
        return FORMULA_OPERATION_EXP;
    } else if (functionString == "uni" || functionString == "uniform") {
        if (numberArguments < 2) {
            cerr << "Error: Uniform distribution needs 2 arguments (min, max)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_UNI;
    } else if (functionString == "unid" || functionString == "uniformd") {
        if (numberArguments < 2) {
            cerr << "Error: Uniform distribution needs 2 arguments (min, max)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_UNID;
    } else if (functionString == "normal") {
        if (numberArguments < 2) {
            cerr << "Error: Earlnag distribution needs 2 arguments (average, deviation)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_NORMAL;
    } else if (functionString == "expdist" || functionString == "exponentialdistribution") {
        return FORMULA_OPERATION_EXP_DISTRIBUTION;
    } else if (functionString == "poisson") {
        return FORMULA_OPERATION_POISSON;
    } else if (functionString == "erlang") {
        if (numberArguments < 2) {
            cerr << "Error: Earlnag distribution needs 2 arguments (lambda, phase)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_ERLANG;
    }

    return FORMULA_OPERATION_NONE;
}
//Tanaka Memo RoutePriority?
AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddVariableUnit(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const string& variableString)
{
    AgentValueFormula::FormulaOperationType operation;

    if (variableString == "_movebypreferedmobilitymeans") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_MODE;

    } else if (variableString == "_arrivaltime") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_ARRIVAL_TIME;

    } else if (variableString == "_totaltraveltime") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRAVEL_TIME;

    } else if (variableString == "_totaltraveldistance") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRAVEL_DISTANCE;

    } else if (variableString == "_totalpublictransportationdelay") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_VARIABILITY_TIME;

    } else if (variableString == "_price") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_PRICE;

    } else if (variableString == "_totaltransfercount") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRANSFER_TIME;

    } else if (variableString == "_totaltransferduration") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRANSFER_DURATION;

    } else if (variableString == "_numberofpeopleonroute") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_PASSANGER_CONGESTION;

    } else if (variableString == "_numberofvehiclesonroute") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_VEHICLE_CONGESTION;

     //} else if (variableString == "_affect") { //Tanaka added

        //operation = ROUTE_COST_START + AGENT_ROUTE_COST_AFFECT;

     //} else if (variableString == "_handicapped") {

      ///  operation = ROUTE_COST_START + AGENT_ROUTE_COST_HANDICAPPED;
       //Tanaka added
    } else if (variableString == "_segmentcongestion") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_HEALTH_FACTOR_CONGESTION;

    } else if (variableString == "_delayforexpectedarrivaltime") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_TRIP_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayforspecfiedarrivaltime") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_ARRIVAL_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayforlastviappoint") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_LAST_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayfornextviapoint") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_NEXT_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_utility1updatecount") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_UTILITY1_COUNTER;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_utility2updatecount") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_UTILITY2_COUNTER;
        assert(IsHealthOrUtilityCost(operation));

    } else {

        if (!parameters.Contains(variableString)) {
            cerr << "Error: invalid variable " << variableString << endl;
            exit(1);
        }

        operation = ConvertTofFormulaOperationType(parameters.GetId(variableString));
        assert((IsAgentStatus(operation)));
    }

    const FormulaUnitIdType unitId = formulaUnits.size();
    formulaUnits.push_back(FormulaUnit(unitId, operation));

    return unitId;
}

AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddFunctionUnitRecursively(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds)
{
    assert(IsFunctionString(aString));

    const FormulaUnitIdType unitId = formulaUnits.size();
    formulaUnits.push_back(FormulaUnit(unitId));

    vector<string> arguments;
    size_t argumentStartPos = 0;

    const size_t arcPos = aString.find_first_of('(');
    const string functionString = aString.substr(0, arcPos);

    argumentStartPos = arcPos + 1;

    size_t currentPos = arcPos;
    int arcNumber = 0;

    while (arcNumber != -1) {
        currentPos = aString.find_first_of("(),", currentPos + 1);
        assert(currentPos != string::npos);

        if (aString[currentPos] == '(') {
            arcNumber++;
        } else if (aString[currentPos] == ')') {
            arcNumber--;
        } else if (arcNumber == 0 && aString[currentPos] == ',') {
            arguments.push_back(
                aString.substr(argumentStartPos, currentPos - argumentStartPos));
            argumentStartPos = currentPos + 1;
        }
    }

    arguments.push_back(aString.substr(argumentStartPos, aString.size() - argumentStartPos - 1));

    const FormulaOperationType operation = GetFormulaOperation(functionString, arguments.size());

    assert(operation < NUMBER_FORMULA_TYPES);

    (*this).GetFormulaUnit(unitId).operation = operation;

    for(size_t i = 0; i < arguments.size(); i++) {
        const string& argument = arguments[i];

        FormulaUnitIdType argUnitId;
        double statusValue = 0;

        argUnitId = (*this).AddFormulaUnitRecursively(
            parameters, simStartTimeSec, argument, charactorIds);

        assert(unitId != argUnitId);
        (*this).GetFormulaUnit(unitId).values.push_back(make_pair(argUnitId, statusValue));
    }

    return unitId;
}

//----------------------------------------------------------------------------------------

static inline
AgentLocationType GetLocationType(const string& locationName)
{
    if (locationName == "randombuilding") {

        return AGENT_LOCATION_RANDOM_BUILDING;

    } else if (locationName == "randompark") {

        return AGENT_LOCATION_RANDOM_PARK;

    } else if (locationName == "randompoi") {

        return AGENT_LOCATION_RANDOM_POI;

    } else if (locationName == "randomintersection") {

        return AGENT_LOCATION_RANDOM_INTERSECTION;

    } else if (locationName == "initiallocation") {

        return AGENT_LOCATION_HOME;

    } else if (locationName == "randomroad") {

        return AGENT_LOCATION_RANDOM_ROAD;
        
    } else if (locationName == "nochange") {
        
        return AGENT_LOCATION_NO_CHANGE;

    } else if (locationName == "none" || locationName.empty()) {

        return AGENT_LOCATION_NONE;

    } else {

        return AGENT_LOCATION_POI;
    }
}

static inline
void GetLocationsById(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& resource,
    const string& idString,
    const set<GisPositionIdType>& ignoreLocationIds,
    vector<GisPositionIdType>& locationCandidatetPositionIds)
{
    locationCandidatetPositionIds.clear();

    map<string, AgentCharactorIdType> notUsed;

    const int gisObjectId = AgentValueFormula(
        LabelMap<AgentStatusIdType, AgentValueFormula>(),
        0,
        idString,
        notUsed).CalculateInt(resource);

    const GisPositionIdType positionId =
        theGisSubsystem.GetPositionId(gisObjectId);

    if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

        if (positionId.type == GIS_AREA) {

            theGisSubsystem.GetBuildingPositionIdsInArea(
                positionId.id, ignoreLocationIds, locationCandidatetPositionIds);

        } else if (positionId.type == GIS_BUILDING ||
                   positionId.type == GIS_PARK ||
                   positionId.type == GIS_PARK) {

            locationCandidatetPositionIds.push_back(positionId);

        } else {
            cerr << "Error: location id MUST be a building, park, POI or area id. id:" << idString << endl;
            exit(1);
        }
    }
}

static inline
void GetLocationsByName(
    const GisSubsystem& theGisSubsystem,
    const map<string, vector<GisPositionIdType> >& locationGroups,
    const string& locationName,
    const set<GisPositionIdType>& ignoreLocationIds,
    const bool searchIntersection,
    vector<GisPositionIdType>& locationCandidatetPositionIds)
{
    locationCandidatetPositionIds.clear();

    typedef map<string, vector<GisPositionIdType> >::const_iterator IterType;

    IterType iter = locationGroups.find(locationName);

    if (iter != locationGroups.end()) {

        const vector<GisPositionIdType>& positionIds = (*iter).second;

        for(size_t i = 0; i < positionIds.size(); i++) {
            const GisPositionIdType& positionId = positionIds[i];

            if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

                if (positionId.type == GIS_AREA) {

                    vector<GisPositionIdType> buildingPositionIds;

                    theGisSubsystem.GetBuildingPositionIdsInArea(
                        positionId.id, ignoreLocationIds, buildingPositionIds);

                    for(size_t j = 0; j < buildingPositionIds.size(); j++) {
                        const GisPositionIdType& buildingPositionId = buildingPositionIds[j];

                        if (ignoreLocationIds.find(buildingPositionId) == ignoreLocationIds.end()) {
                            locationCandidatetPositionIds.push_back(buildingPositionId);
                        }
                    }

                } else {
                    locationCandidatetPositionIds.push_back(positionId);
                }
            }
        }

    } else {

        const GisPositionIdType positionId =
            theGisSubsystem.GetPosition(locationName);

        if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

            if (searchIntersection) {

                if (positionId.type == GIS_INTERSECTION) {
                    locationCandidatetPositionIds.push_back(positionId);

                } else {
                    cerr << "Error: pass location MUST be an intersection name. name:" << locationName << endl;
                    exit(1);
                }

            } else {

                if (positionId.type == GIS_AREA) {

                    theGisSubsystem.GetBuildingPositionIdsInArea(
                        positionId.id, ignoreLocationIds, locationCandidatetPositionIds);

                } else if (positionId.type == GIS_BUILDING ||
                           positionId.type == GIS_PARK ||
                           positionId.type == GIS_POI) {

                    locationCandidatetPositionIds.push_back(positionId);

                } else {
                    cerr << "Error: location MUST be a building, park, POI or area name. name:" << locationName << endl;
                    exit(1);
                }
            }
        }
    }
}

TimeType AgentTask::GetStartTime(const AgentResource& resource) const
{
    if (startTime.IsNull()) {
        return ZERO_TIME;
    }

    return startTime.CalculateTime(resource);
}

bool AgentTask::SatisfyCondition(const AgentResource& resource) const
{
    for(size_t i = 0; i < conditionCheckers.size(); i++) {
        if (!conditionCheckers[i].Check(resource)) {
            return false;
        }
    }

    return true;
}

void AgentTask::GetDestinationId(
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const bool ignoreLastPositionFromCandidate,
    GisPositionIdType& positionId,
    bool& isMultipleDestinations,
    AgentResource& resource) const
{
    profileAndTaskTablePtr->GetLocationId(
        theAgentGisPtr,
        destination,
        ignoreLastPositionFromCandidate,
        false/*searchIntersection*/,
        positionId,
        isMultipleDestinations,
        resource);
}

void AgentTask::GetPassVertexIds(
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    deque<VertexIdType>& passVertexIds,
    AgentResource& resource) const
{
    passVertexIds.clear();

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    for(size_t i = 0; i < passIntersectionLocationInfos.size(); i++) {

        GisPositionIdType positionId;
        bool isMultipleDestinations;
        

        profileAndTaskTablePtr->GetLocationId(
            theAgentGisPtr,
            passIntersectionLocationInfos[i],
            true/*ignoreLastPositionFromCandidate*/,
            true/*searchIntersection*/,
            positionId,
            isMultipleDestinations,
            resource);

        if (positionId.type != GIS_INTERSECTION) {
            cerr << "Specify valid intersection name for pass intersection names" << endl;
            exit(1);
        }

        passVertexIds.push_back(
            subsystem.GetIntersection(positionId.id).GetVertexId());
    }
}

void AgentTaskTable::GetInitialLocationId(
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    GisPositionIdType& positionId,
    AgentResource& resource) const
{
    bool isMultiplePositionsNotUsed;

    (*this).GetLocationId(theAgentGisPtr, initialLocation, positionId, isMultiplePositionsNotUsed, resource);
}

void AgentTaskTable::GetLocationId(
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const LocationInfo& locationInfo,
    GisPositionIdType& positionId,
    bool& isMultiplePositions,
    AgentResource& resource) const
{
    profileAndTaskTablePtr->GetLocationId(
        theAgentGisPtr,
        locationInfo,
        true/*ignoreLastPositionFromCandidate*/,
        false/*searchIntersection*/,
        positionId,
        isMultiplePositions,
        resource);
}

// Umeki add function
void ShelterMergeSort(EscapeAgentInfo* escapeAgentInfo, KnapsackInfo knapsackInfo, ShelterInfo* shelterInfo)
{
  cout << "ShelterMergeSort-------------" << endl;
  for(int i=0; i < knapsackInfo.getNumberAgent(); i++){ //各避難所ごとのAgent利得リストの取得
    for(int j=0; j < knapsackInfo.getNumberShelter(); j++){
      shelterInfo[j].setAgentGain(escapeAgentInfo[i].getAgentGain(j));
      shelterInfo[j].setAgentId(escapeAgentInfo[i].getAgentId());
      if (escapeAgentInfo[i].getAgentResource().GetProfileName() == "Agent") {
      //shelterInfo[j].setAgentProfile(escapeAgentInfo[i].getAgentResource().GetProfileName()); //Tanaka added
      shelterInfo[j].setAgentProfile(0);
       } else {
      shelterInfo[j].setAgentProfile(1); //Tanaka added
      }
    }
  }
   cout << "before------------" << endl;
   for(int i=0; i < knapsackInfo.getNumberShelter(); i++)
     for(int j=0; j < knapsackInfo.getNumberAgent(); j++){
        cout << "shelterInfo[" << i << "].getShelterGain[" << j << "]:" << shelterInfo[i].getAgentGain(j) << "\tsuffix:" << shelterInfo[i].getAgentId(j) << endl;
        cout << "Profile=" << shelterInfo[i].getAgentProfile(j) << endl; //Tanaka added
    }

  for(int j=0; j < knapsackInfo.getNumberShelter(); j++){ //各避難所ごとのAgent利得リストのソーティング
    cout << "================================" << knapsackInfo.getNumberAgent() << endl;
    shelterInfo[j].initializeTemp(knapsackInfo.getNumberAgent());
    shelterInfo[j].MergeSort(0, knapsackInfo.getNumberAgent() - 1);
  }

  cout << "after Result------------" << endl; //commen
  for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
    cout << endl;
    for(int j=0; j < knapsackInfo.getNumberAgent(); j++){
       cout << "shelterInfo[" << i << "].getShelterGain[" << j << "]:" << shelterInfo[i].getAgentGain(j) << "\tAgentId:" << shelterInfo[i].getAgentId(j) << endl;
       cout << "Profile=" << shelterInfo[i].getAgentProfile(j) << endl; //Tanaka added
   }
  }

}


//EscapeAgentInfo DecideShelter(float* splitShel, int* priorityList, ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo){
double DecideShelter(ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo, int* decidedAgentId, KnapsackInfo knapsackInfo){
    //cout << shel[0] << endl;
    cout << "DecideShelter----------" << endl;
    int iAgentId = 0;
    int jShelterNumber = 0;
    GisPositionIdType ObjectShelterId;
    float maxGain = -10000000000;
    float totalGain = 0;
    bool limitFlag = false;

    for(int i=0; i < knapsackInfo.getNumberAgent(); i++){
      cout << i << "人目の避難先決定" << endl;
      for(int j=0; j < knapsackInfo.getNumberShelter(); j++){ //各避難所の一番大きい利得を比較
        int agentNumber = shelterInfo[j].getSuffix();                                                                                                                                                                                                                         
        if(shelterInfo[j].checkShelter() == true){  //このシェルターがすでに満杯じゃないかの判定
          cout << j+1 << "番目" << endl;
          while(decidedAgentId[shelterInfo[j].getAgentId(agentNumber)] == 1){ //その避難者が避難場所未決定かを判定
            cout << "----check-----agentNumber:" << shelterInfo[j].getAgentId(agentNumber) << endl;
            shelterInfo[j].addSuffix();
            agentNumber = shelterInfo[j].getSuffix();
          }
          float shelterGain = shelterInfo[j].getAgentGain( agentNumber ); //利得取得

          if( maxGain < shelterGain ){  //一番大きい利得判定
            maxGain = shelterGain;
            iAgentId = shelterInfo[j].getAgentId(agentNumber);
            jShelterNumber = j;
            ObjectShelterId = shelterInfo[j].getShelterId();
          }

          limitFlag = true;

        } else {
          cout << "shelterId:" << shelterInfo[j].getShelterId().id << "はすでに埋まっています" << endl;
        }

      }

      if(limitFlag){  //
        decidedAgentId[iAgentId] = 1;
        if(escapeAgentInfo[iAgentId].getNotObeyFlag() == false){
            shelterInfo[jShelterNumber].addPerson();
        }
        shelterInfo[jShelterNumber].addSuffix();
        escapeAgentInfo[iAgentId].setShelterId(ObjectShelterId);
        escapeAgentInfo[iAgentId].setShelterPrefix(jShelterNumber);
        escapeAgentInfo[iAgentId].setPurposeShelterGain(maxGain);
        //cout << maxGain << "\tid:" << iAgentId << "\tObject:" << ObjectShelterId.id << endl;
        totalGain += maxGain;
        for(int z = 0; z < knapsackInfo.getNumberShelter(); z++){
          cout << "shelterInfo[" << z << "]\tcurrentPersonNumber:" << shelterInfo[z].getCurrentPerson() <<"\tAllowableValue:" << shelterInfo[z].getAllowableValue() << endl;
        }
        for(int z=0; z < knapsackInfo.getNumberAgent(); z++){
          cout << "decidedAgentId[" << z << "]:" << decidedAgentId[z] << endl;
        }
      } else {
        cout << "全ての収容所が満杯になりました" << endl;

        break;
      }
      limitFlag =false;
      maxGain = -10000000000;
      //if(i == 4) exit(1);
      // cout << "shelterNumber:" << jShelterNumber << "\tObjectShelterId:" << shelterInfo[escapeAgentInfo[iAgentId].getShelterId()].getShelterId() << "\tAgentId:" << iAgentId << "\tmaxGain:" << maxGain <<  endl;
      // //cout << "shelterInfo[" << iShelterNumber << "].currentPersonNumber:" << shelterInfo[iShelterNumber].currentPersonNumber << endl;
      // for(int z = 0; z < knapsackInfo.getNumberShelter(); z++){
      //   cout << "shelterInfo[" << z << "]\tcurrentPersonNumber:" << shelterInfo[z].getCurrentPerson() <<"\tAllowableValue:" << shelterInfo[i].getAllowableValue() << endl;
      // }
    }

    return totalGain;

}

void knapasckSimple(ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo, KnapsackInfo knapsackInfo){
    //cout << "knapasckSimple-------" << endl;
    ShelterMergeSort(escapeAgentInfo, knapsackInfo, shelterInfo); // 避難所ごとに避難時間をソーティングする
    const int n = knapsackInfo.getNumberAgent();
    int decidedAgentId[n];  //可変長の配列だと，{}初期化が使えなかったため，下記のようにする
//    for(int i=0; i < n; i++){
//        if(escapeAgentInfo[i].getNotObeyFlag() == false) decidedAgentId[i] = 0; //knapsackにしたがう
//        else if(escapeAgentInfo[i].getNotObeyFlag() == true) decidedAgentId[i] = 1; //従わない
//    }
    for(int i=0; i < n; i++){
        decidedAgentId[i] = 0;
    }
    float totalGain;

    totalGain = DecideShelter(shelterInfo, escapeAgentInfo, decidedAgentId, knapsackInfo);

    if(debugMode == true){
        cout << "Result-----------------------" << endl;
        for(int i=0; i < n; i++){
          cout << "escapeAgentInfo[" << i << "]\tagentId:" << escapeAgentInfo[i].getObjectAgentId() <<"\tshelterNumber:" << escapeAgentInfo[i].getShelterId().id
                << "\tObjectshelterGain:" << escapeAgentInfo[i].getPurposeShelterGain() << endl;
        }
        for(int i = 0; i < knapsackInfo.getNumberShelter(); i++){
          cout << "shelterInfo[" << i << "]\tcurrentPersonNumber:" << shelterInfo[i].currentPersonNumber <<"\tAllowableValue:" << shelterInfo[i].getAllowableValue() << endl;
        }
        cout << "totalGain:" << totalGain << endl;
    }
    //Step 1.全ての避難者の各避難場所までの予測避難時間リストを作成する
    //Step 2.避難場所ごとに各避難者の避難時間が短い時間順にソートする
    //Step 3.各避難所への避難時間が一番短い時間を各避難所ごとに比較し，その中で一番小さい時間のものを採用(そのAgentの避難先が決定)し，その避難所に避難者数を１増加させ，避難者の避難先未決定リストの中からAentIdを削除する
        //Step 4.避難所への避難者数が収容数を超えた場合，避難先リストから除外
    //Step 5.避難者の避難先未決定リストが０になるまで繰り返す
}
// add above


void AgentProfileAndTaskTable::GetLocationId(
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const LocationInfo& locationInfo,
    const bool ignoreLastPositionFromCandidate,
    const bool searchIntersection,
    GisPositionIdType& positionId,
    bool& isMultiplePositions,
    AgentResource& resource) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const AgentLocationType locationType = GetLocationType(locationInfo.locationName);

    const GisPositionIdType currentPositionId = resource.PositionId();

    set<GisPositionIdType> ignoredDestinationIds = resource.UnreachableDestinationIds();
    
//    std::cout << "\nAgemtProfileAndTaskTable::GetLocationId-----------" << resource.AgentId() << std::endl;
    //std::cout << "locationInfo:" << locationInfo.locationChoiceType << std::endl;
    //std::cout << "locationType:" << locationType << std::endl;
    //cout << "locationInfoName:" << locationInfo.locationName << endl;
    //std::cout << "isMultiplePositions:" << (isMultiplePositions ? "True" : "False" ) << std::endl;

    
    Vertex currentPositionU = theAgentGisPtr->GetVertex(currentPositionId.id);
    
    //std::cout << "currentPositionId.id=" << currentPositionId.id << std::endl;
    //std::cout << "currentPositionId.type=" << currentPositionId.type << std::endl;
    //std::cout << "currentPosition.x=" << currentPositionU.x << std::endl;
    //std::cout << "currentPosition.y=" << currentPositionU.y << std::endl;
    
    
    if (ignoreLastPositionFromCandidate) {
        ignoredDestinationIds.insert(currentPositionId);
    }

    vector<GisPositionIdType> locationCandidateIds;

    HighQualityRandomNumberGenerator& randomNumberGeneratorForDestinationChoice =
        resource.GetRandomNumberGeneratorForDestinationChoice();
    
    if (locationType == AGENT_LOCATION_NONE) {

        locationCandidateIds.push_back(currentPositionId);
//        cout << "None" << endl;

    } else if (locationType == AGENT_LOCATION_RANDOM_BUILDING) {
//        cout << "Building" << endl;
        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_BUILDING,
            ignoredDestinationIds,
            randomNumberGeneratorForDestinationChoice,
            found,
            locationId);
        
        //debug by umeki
        //std::cout << "AgentProfileAndTaskTable::GetLocationId_RANDOM_BUILDING::" << locationId.id <<  std::endl;
        
        locationInfo.locationChoiceType;


        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no building for random building specification." << endl;
            exit(1);
        }
        //Umeki write
    } else if (locationType == AGENT_LOCATION_RANDOM_ROAD) {
//        cout << "Road" << endl;

        bool found;
        GisPositionIdType locationId;
        
        subsystem.GetARandomPosition(
            GIS_ROAD,
            ignoredDestinationIds,
            randomNumberGeneratorForDestinationChoice,
            found,
            locationId);
        
        //debug by umeki
        //std::cout << "GetLocationId_RANDOM_ROAD:locationId;;" << locationId.id << std::endl;
        
//        cout << resource.AgentId() << "RandomRoad::::" << locationInfo.locationChoiceType <<
//                "\t::" << resource.GetProfileName() << endl;

        if (found) {
            locationCandidateIds.push_back(locationId);
            // debug by umeki
            //for(int i = 0; i < locationCandidateIds.size(); i++){
            //    std::cout << "foundlocationCandidateIds[" << i << "]:" << locationCandidateIds[i].id << "::size:" << locationCandidateIds.size() << std::endl;
            //}
        } else {
            cerr << "There is no road for random road specification." << endl;
            exit(1);
        }
        
    } else if (locationType == AGENT_LOCATION_RANDOM_PARK) {
//        cout << "Park" << endl;

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_PARK,
            ignoredDestinationIds,
            randomNumberGeneratorForDestinationChoice,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no park for random park specification." << endl;
            exit(1);
        }

    } else if (locationType == AGENT_LOCATION_RANDOM_POI) {
//        cout << "POI" << endl;

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_POI,
            ignoredDestinationIds,
            randomNumberGeneratorForDestinationChoice,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no POI for random POI specification." << endl;
            exit(1);
        }

    } else if (locationType == AGENT_LOCATION_RANDOM_INTERSECTION) {
//        cout << "Intersection" << endl;

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_INTERSECTION,
            ignoredDestinationIds,
            randomNumberGeneratorForDestinationChoice,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no intersection for random intersection specification." << endl;
            exit(1);
        }

    } else  if (locationType == AGENT_LOCATION_HOME) {
//        cout << "Home" << endl;
        const GisPositionIdType homePositionId = resource.HomePositionId();

        if (homePositionId == GisPositionIdType()) {
            assert(false);
            cerr << "There is no InitilLocation position." << endl;
            exit(1);
        }

        locationCandidateIds.push_back(homePositionId);

    } else { //LocationCandidateIdsの取得(それ以外の機能は要調査)

        if (locationInfo.isId) {

            GetLocationsById(
                subsystem,
                resource,
                locationInfo.locationName,
                ignoredDestinationIds,
                locationCandidateIds);
        } else {   
            if (thePublicVehicleTablePtr->ContainsLine(locationInfo.locationName)) {
                locationCandidateIds.push_back(
                    GisPositionIdType(
                        GIS_VEHICLE_LINE,
                        thePublicVehicleTablePtr->GetLineId(locationInfo.locationName)));
            } else {

                GetLocationsByName(
                    subsystem,
                    locationGroups,
                    locationInfo.locationName,
                    ignoredDestinationIds,
                    searchIntersection,
                    locationCandidateIds);
            }
        }
        if(decideFlag == false && countFlag == true){
            cout << "SettingKnapsackParam()" << endl;
            SettingKnapsackParam(
                subsystem,
                locationCandidateIds,
                fileFlag);
            decideFlag = true;
            cout << "start Simulation----------------------abnormalAgentCount::" << abnormalCount << endl;
        }
        
    }

    if(agentAddCount > 1 && (escapeAgentInfoTemp[0].getAgentResource().AgentId() == resource.AgentId()) && countFlag == false){
        countFlag = true;
        cout << "Initial_agentCount:" << agentAddCount << endl;
        cout << "Initial_shelterCount:" << locationCandidateIds.size() << endl;
    }
    if(countFlag == false){
            // Only input AgentStatus(ID,resource)
        bool notObeyFlag = false;
        //Tanaka Memo Nov21st&& resource.GetProfileName() != "Slow" 
        if(resource.GetProfileName() != "Agent") {
           if(resource.GetProfileName() != "Slow") {
            notObeyFlag = true;
            abnormalCount++;
         }
        }
        SettingAgentParam(
            resource,
            notObeyFlag);

    }
    
    if (locationCandidateIds.empty()) {
        positionId = UNREACHABLE_POSITION_ID;
        isMultiplePositions = false;
        return;
    }

    if (locationCandidateIds.size() == 1) {
        if (ignoredDestinationIds.find(locationCandidateIds.front()) != ignoredDestinationIds.end()) {
            positionId = UNREACHABLE_POSITION_ID;
        } else {
            positionId = locationCandidateIds.front();
        }

        isMultiplePositions = false;
        return;
    }

    isMultiplePositions = true;

    //ここで避難所情報を収集
    if (locationInfo.locationChoiceType == AGENT_LOCATION_CHOICE_NEAREST) {
//        std::cout << "locationInfo.LocationChoiceType:::Choice_Nearest"<< std::endl;
        
        
        if(decideFlag == false && countFlag == true){    //避難場所決定
            if(debugMode == true){
                for(int i=0; i < agentAddCount; i++){
                    std::cout << "Input Agent:" << escapeAgentInfoTemp[i].agentId << "\tcount" << 
                            escapeAgentInfoTemp[i].getAgentResource().AgentId() <<  std::endl;
                }
            }
            int shelterTotal = 0;
            if(locationCandidateIds.size() == 0) shelterTotal = 1;
            else shelterTotal = locationCandidateIds.size();
            KnapsackInfo knapsackInfo(agentAddCount, shelterTotal); //AgentとShelterの総数を管理する
            ShelterInfo shelterInfo[knapsackInfo.getNumberShelter()];
            SettingShelParam(
                    subsystem,
                    locationCandidateIds,
                    shelterInfo,
                    escapeAgentInfoTemp,
                    knapsackInfo,
                    fileFlag
                    );
            if(debugMode){
                cout << "input AgentData " << endl;
                for(int i=0; i < knapsackInfo.getNumberAgent(); i++){
                  //escapeAgentInfo[i].setObjectShelterId();  //Scenargieにて実装
                  for(int j=0; j < knapsackInfo.getNumberShelter(); j++){
                    cout << "escapeAgentInfo[" << i << "]:\t" << j << "\tshelterGain:" << escapeAgentInfoTemp[i].getAgentGain(j) << endl;
                  }
                }
            }
            cout << "input shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
            
            knapasckSimple(shelterInfo, escapeAgentInfoTemp, knapsackInfo); //自作シンプル
            
            cout << "result shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
  
            cout << "------Start Simulation--------------abnormalAgentCount::" << abnormalCount << endl;
            
            decideFlag = true;  //避難場所が決定した場合の処理
        }
        
        
        double minDistance = DBL_MAX;

        for(size_t i = 0; i < locationCandidateIds.size(); i++) {

            const Vertex position = subsystem.GetPositionVertex(locationCandidateIds[i]);

            double distance = resource.Position().DistanceTo(position);

            if (distance < minDistance) {
                minDistance = distance;
                positionId = locationCandidateIds[i];
            }
        }
        
        if(moveFlag == true){
            //cout << "taraimawasisisisisisi::::" << resource.AgentId() << "!!!!!!" << endl;
            int currentAgentId = resource.AgentId();
            escapeAgentInfoTemp[currentAgentId - minAgentId].addTurnOffCount();
            //cout << "AgentId;;" << currentAgentId << "\tTurnCount::" << escapeAgentInfoTemp[currentAgentId - minAgentId].getTurnOffCount() << endl;
        }

    } else if (locationInfo.locationChoiceType == AGENT_LOCATION_CHOICE_RANDOM) {
//        std::cout << "locationInfo.LocationChoiceType:::Choice_Random"<< std::endl;   //ここは初期位置もランダム設定する
        Vertex positionU = theAgentGisPtr->GetVertex(resource.HomePositionId().id);
       
        
        
        if(decideFlag == false && countFlag == true){    //避難場所決定
            if(debugMode){
                for(int i=0; i < agentAddCount; i++){
                    std::cout << "Input Agent:" << escapeAgentInfoTemp[i].agentId << "\tcount" << 
                            escapeAgentInfoTemp[i].getAgentResource().AgentId() <<  std::endl;
                }
            }
            int shelterTotal = 0;
            if(locationCandidateIds.size() == 0) shelterTotal = 1;
            else shelterTotal = locationCandidateIds.size();
            KnapsackInfo knapsackInfo(agentAddCount, shelterTotal); //AgentとShelterの総数を管理する
            ShelterInfo shelterInfo[knapsackInfo.getNumberShelter()];
            cout << "agentCount::" << knapsackInfo.getNumberAgent() << "\tshelterCount::" << knapsackInfo.getNumberShelter() << endl;
            SettingShelParam(
                    subsystem,
                    locationCandidateIds,
                    shelterInfo,
                    escapeAgentInfoTemp,
                    knapsackInfo,
                    fileFlag
                    );
            
            if(debugMode){
                cout << "input AgentData " << endl;
                for(int i=0; i < knapsackInfo.getNumberAgent(); i++){
                  //escapeAgentInfo[i].setObjectShelterId();  //Scenargieにて実装
                  for(int j=0; j < knapsackInfo.getNumberShelter(); j++){
                    cout << "escapeAgentInfo[" << i << "]:\t" << j << "\tshelterGain:" << escapeAgentInfoTemp[i].getAgentGain(j) << endl;
                  }
                }
            }
            cout << "input shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
            
            knapasckSimple(shelterInfo, escapeAgentInfoTemp, knapsackInfo); //自作シンプル
            
            cout << "result shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
  
            cout << "------Start Simulation--------------abnormalAgentCount::" << abnormalCount<< endl;
            
            decideFlag = true;  //避難場所が決定した場合の処理
        }
        
        //umeki write
        for(int i=0; i < locationCandidateIds.size(); i++){
            const Vertex position = subsystem.GetPositionVertex(locationCandidateIds[i]);
            double distance = resource.Position().DistanceTo(position);
            //std::cout << "ObjectName::" << subsystem.GetGisObject(position).GetObjectName() << endl;
            //std::cout << "locationCandidateIds[" << locationCandidateIds[i].type << "]::" << locationCandidateIds[i].id << "\tlocationDistance::" << distance << endl;
        }
        
        if(randomSeedFlag){
        //random Seed by time
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_int_distribution<> rand_t(0, (int32_t)(locationCandidateIds.size() - 1));
            positionId = locationCandidateIds[rand_t(mt)];
        }
        else{
        // umeki write above
            positionId = locationCandidateIds[
                randomNumberGeneratorForDestinationChoice.GenerateRandomInt(
                    0, (int32_t)(locationCandidateIds.size() - 1))];
        }
        
        
        if(moveFlag == true){
            //cout << "taraimawasisisisisisi::::" << resource.AgentId() << "!!!!!!" << endl;
            int currentAgentId = resource.AgentId();
            escapeAgentInfoTemp[currentAgentId - minAgentId].addTurnOffCount();
            //cout << "AgentId;;" << currentAgentId << "\tTurnCount::" << escapeAgentInfoTemp[currentAgentId - minAgentId].getTurnOffCount() << endl;
        }


    } else if(locationInfo.locationChoiceType == AGENT_LOCATION_CHOICE_KNAPSACK){
//        std::cout << "locationInfo.LocationChoiceType:::Choice_Knapsack"<< resource.AgentId() << std::endl;
        //To write in a later time by Umeki
        ifstream ifs("./result.csv");
        
        int agentCountCsv = -1;
        
        
        if(ifs.is_open() && fileFlag == false){
            cout << "atta" << endl;
            fileFlag = true;
            string filename = "./result.csv";
            std::ifstream reading_file;
            reading_file.open(filename, std::ios::in);

            std::string reading_line_buffer;

            std::cout << "reading " << filename << "..." << std::endl;

            while (!reading_file.eof())
            {
                // read by line
                std::getline(reading_file, reading_line_buffer);

                std::cout << reading_line_buffer << std::endl;

                // read by delimiter on reading "one" line
                const char delimiter = ' ';
                std::string separated_string_buffer;
                std::istringstream line_separater(reading_line_buffer);
                std::getline(line_separater, separated_string_buffer, delimiter);

                const char *delim = ",";
                char *words[100];
                char *cp = new char[separated_string_buffer.size() + 1];
                std::strcpy(cp, separated_string_buffer.c_str());
                int len = 0;
                for(len = 0; len < 100; len++){
                    if((words[len] = strtok(cp, delim)) == NULL){
                        break;
                    }
                    cp = NULL;
                }
                delete[] cp;
                vector<string> wordsList;
                for(int i=0; i<len; i++){   //Char*[]をvector<string>に変換
                    //puts(words[i]);
                    string wordsStr = string(words[i]);
                    wordsList.push_back(wordsStr);
                }
                for(int i=0; i < wordsList.size();  i++){
                    //cout << "List" << wordsList[i] << endl;
                    if(i == 0){
                        agentCountCsv++; //csv内に何人の有効データが存在していたかカウント
                        if(agentCountCsv > 0){
                            escapeAgentInfoTemp[agentCountCsv - 1].clearAgentGain();
                        }
                    } 
                    if(agentCountCsv > 0 && i > 5){  //ヘッダ情報の読み飛ばしと前六つのパラメータ読み飛ばし
//                        cout << "csv:" << escapeAgentInfoTemp[agentCountCsv - 1].checkAgentGainNumber()
//                                << "\tAddCount:" << wordsList.size() - 5 << "NumberOfAgentNow" << agentCountCsv <<  endl;
                        assert(escapeAgentInfoTemp[agentCountCsv - 1].getObjectAgentId() == stoi(wordsList[0]));
                        escapeAgentInfoTemp[agentCountCsv - 1].setAgentGain(stod(wordsList[i]));
                    }
                }
                //cout << "CSVcount:" << escapeAgentInfoTemp[agentCountCsv - 1].checkAgentGainNumber() <<
                //        "\tcsv:" << (locationCandidateIds.size()) << "NumberOfAgentNow" << agentCountCsv << endl;
                if(agentCountCsv > 0){
                    cout << "shelterNumber_csv::" << escapeAgentInfoTemp[agentCountCsv - 1].checkAgentGainNumber()
                            << "\tshleterNumber::" << locationCandidateIds.size() << "\troop_number::" << agentCountCsv <<  endl;
                    for(int i=0; i < escapeAgentInfoTemp[agentCountCsv - 1].checkAgentGainNumber(); i++){
                        cout << "shelterNumber" << i << ":\t" << escapeAgentInfoTemp[agentCountCsv - 1].getAgentGain(i) << endl;;
                    }
                    assert(escapeAgentInfoTemp[agentCountCsv - 1].checkAgentGainNumber() == locationCandidateIds.size());
                }
            }
            if(agentCountCsv != agentAddCount){
                cout << "csv:" << agentCountCsv << "\tAddCount:" << agentAddCount << endl;
                assert(agentCountCsv == agentAddCount); //Csv内の人数と実際のシナリオの人数が一致しているか確認
            }
            if(decideFlag)  cout << "decideFlag::true" << endl;
            else cout << "decideFlag::false" << endl;
            decideFlag = false;
//            exit(1);
        }
        


        if(decideFlag == false && countFlag == true){    //避難場所決定
//            for(int i=0; i < agentAddCount; i++){
//                std::cout << "Input Agent:" << escapeAgentInfoTemp[i].agentId << "\tcount" << 
//                        escapeAgentInfoTemp[i].getAgentResource().AgentId() <<  std::endl;
//            }
            int shelterTotal = 0;
            if(locationCandidateIds.size() == 0) shelterTotal = 1;
            else shelterTotal = locationCandidateIds.size();
            KnapsackInfo knapsackInfo(agentAddCount, shelterTotal); //AgentとShelterの総数を管理する
            ShelterInfo shelterInfo[knapsackInfo.getNumberShelter()];
            SettingShelParam(
                    subsystem,
                    locationCandidateIds,
                    shelterInfo,
                    escapeAgentInfoTemp,
                    knapsackInfo,
                    fileFlag
                    );
            if(debugMode){
                cout << "input AgentData " << endl;
                for(int i=0; i < knapsackInfo.getNumberAgent(); i++){
                  //escapeAgentInfo[i].setObjectShelterId();  //Scenargieにて実装
                  for(int j=0; j < knapsackInfo.getNumberShelter(); j++){
                    cout << "escapeAgentInfo[" << i << "]:\t" << j << "\tshelterGain:" << escapeAgentInfoTemp[i].getAgentGain(j) << endl;
                  }
                }
            }
            cout << "input shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
            
            knapasckSimple(shelterInfo, escapeAgentInfoTemp, knapsackInfo); //自作シンプル

            cout << "result shelterInfo" << endl;
            for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
              cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
                  << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
            }
  
            cout << "------Start Simulation--------------abnormalAgentCount::" << abnormalCount << endl;
            
            
            decideFlag = true;  //避難場所が決定した場合の処理
        }


        bool foundPosition = false;
        for(int i=0; i < agentAddCount; i++){
            if(escapeAgentInfoTemp[i].getObjectAgentId() == resource.AgentId()){
                positionId = escapeAgentInfoTemp[i].getShelterId();
                foundPosition = true;
            }
        }
        
        
        if(resource.GetProfileName() != "Agent" || moveFlag == true) {
            if(resource.GetProfileName() != "Slow" || moveFlag == true) {
            double minDistance = DBL_MAX;
            for(size_t i = 0; i < locationCandidateIds.size(); i++) {
                const Vertex position = subsystem.GetPositionVertex(locationCandidateIds[i]);
                double distance = resource.Position().DistanceTo(position);
                if (distance < minDistance) {
                    minDistance = distance;
                    positionId = locationCandidateIds[i];
                }
            }
            foundPosition = true;
            
            if(moveFlag == true){   //たらい回し発生
                //cout << "taraimawasisisisisisi::::" << resource.AgentId() << "!!!!!!" << endl;
                int currentAgentId = resource.AgentId();
                escapeAgentInfoTemp[currentAgentId - minAgentId].addTurnOffCount();
                //cout << "AgentId;;" << currentAgentId << "\tTurnCount::" << escapeAgentInfoTemp[currentAgentId - minAgentId].getTurnOffCount() << endl;
            }
          }
        } 

        
        if(foundPosition == false){
            std::cout << "Not Found Evacuation Shelter" << endl;
            std::cout << "AgentId::" << resource.AgentId() << endl;
            std::cout << "agentAddCount:" << agentAddCount << endl;
            exit(1);
        }
    }
        

    
//    std::cout << "GetLocationId----End-----" << std::endl;
    
}

//make this part by Umeki
void AgentProfileAndTaskTable::SettingAgentParam(
        const AgentResource& resource,
        bool notObeyFlag
        ) const
{
//    cout << "SettingAgent" << endl;
//    std::cout << "Input Agent:" << resource.AgentId() << "\tcount" << agentAddCount <<  std::endl;
    agentAddCount++;
    escapeAgentInfoTemp[agentAddCount - 1].setObjectAgentId(resource.AgentId());
    escapeAgentInfoTemp[agentAddCount - 1].agentId = agentAddCount - 1;
    escapeAgentInfoTemp[agentAddCount - 1].setAgentResource(resource);
    escapeAgentInfoTemp[agentAddCount - 1].setNotObeyFlag(notObeyFlag);
    if(minAgentId == 0) minAgentId = resource.AgentId();
    if(minAgentId > resource.AgentId()){ //最小のAgentIdを取得(AgentIdは連番であることが前提)
        // escapeAgentInfoTemp[minAgentId - currentAgentId]で現状のAgentを取得するため
        minAgentId = resource.AgentId();
    }

}

void AgentProfileAndTaskTable::SettingShelParam(
        const GisSubsystem& subsystem,
        vector<GisPositionIdType> locationCandidateIds,
        ShelterInfo* shelterInfo,
        EscapeAgentInfo* escapeAgentInfo,
        KnapsackInfo knapsackInfo,
        bool fileFlag
        ) const
{
//    cout << "SettingShel" << endl;


    //std::cout << "Input Agent:" << resource.AgentId() << "\tcount" << agentAddCount <<  std::endl;
    std::cout << "Input ShelterNumber" << knapsackInfo.getNumberShelter() << std::endl;

    for(size_t i = 0; i < knapsackInfo.getNumberShelter(); i++) {

        const Building building_ = subsystem.GetBuilding(locationCandidateIds[i].id);
        if(marginFlag){
            shelterInfo[i].setAllowableValue(building_.GetHumanCapacity() * (1.0 - marginRate) + 1 );
        }
        else if(marginFlag == false){
            shelterInfo[i].setAllowableValue(building_.GetHumanCapacity());
        }
        shelterInfo[i].setShelterId(locationCandidateIds[i]);
        const Vertex position = subsystem.GetPositionVertex(locationCandidateIds[i]);

        //const GisObject test = subsystem.GetGisObject(locationCandidateIds[i]);
        //std::cout << "Name:" << test.GetObjectId() << std::endl;
        std::cout << "capacity:" << building_.GetHumanCapacity() << std::endl;   

        if(fileFlag == false){
            for(int j = 0; j < knapsackInfo.getNumberAgent(); j++){

                AgentResource resource = escapeAgentInfo[j].getAgentResource();
                double WalkHumanSpeed = resource.WalkSpeedMetersPerSec();
                double distance = resource.Position().DistanceTo(position);
                double Gain = -1.0 * (distance/WalkHumanSpeed);
                escapeAgentInfo[j].setAgentGain(Gain);
            }
        }

    }
}

void AgentProfileAndTaskTable::SettingKnapsackParam(
        const GisSubsystem& subsystem,
        vector<GisPositionIdType> locationCandidateIds,
        bool fileFlag
        ) const
{
    int shelterTotal = 0;
    if(locationCandidateIds.size() == 0) shelterTotal = 1;
    else shelterTotal = locationCandidateIds.size();
    KnapsackInfo knapsackInfo(agentAddCount, shelterTotal); //AgentとShelterの総数を管理する
    ShelterInfo shelterInfo[knapsackInfo.getNumberShelter()];
    SettingShelParam(
            subsystem,
            locationCandidateIds,
            shelterInfo,
            escapeAgentInfoTemp,
            knapsackInfo,
            fileFlag
            );
    cout << "input AgentData " << endl;
    if(debugMode == true){
        for(int i=0; i < knapsackInfo.getNumberAgent(); i++){
          //escapeAgentInfo[i].setObjectShelterId();  //Scenargieにて実装
          for(int j=0; j < knapsackInfo.getNumberShelter(); j++){
            cout << "escapeAgentInfo[" << i << "]:\t" << j << "\tshelterGain:" << escapeAgentInfoTemp[i].getAgentGain(j) << endl;
          }
        }
        cout << "input shelterInfo" << endl;
        for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
          cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
              << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
        }
    }

    knapasckSimple(shelterInfo, escapeAgentInfoTemp, knapsackInfo); //自作シンプル
    
    cout << "result shelterInfo" << endl;
//    if(debugMode == true){
        for(int i=0; i < knapsackInfo.getNumberShelter(); i++){
          cout << "shelterInfo[" << i << "]:\tshelterId:" << shelterInfo[i].getShelterId().id << "\tAllowableValue:" << shelterInfo[i].getAllowableValue()
              << "\tcurrentPerson:" << shelterInfo[i].currentPersonNumber << endl;
//        }
        //decideFlag = true;  //避難場所が決定した場合の処理
    }
}


TimeType AgentTask::GetEndTime(const AgentResource& resource) const
{
    if (endTime.IsNull()) {
        return INFINITE_TIME;
    }

    return endTime.CalculateTime(resource);
}

TimeType AgentTask::GetWaitTime(const AgentResource& resource) const
{
    if (waitTime.IsNull()) {
        return ZERO_TIME;
    }

    return waitTime.CalculateTime(resource);
}

void AgentTask::GetTimeLine(
    const AgentResource& resource,
    const TimeType& earlyStartTime,
    TimeToSearchRoute& timeToSearchRoute) const
{
    timeToSearchRoute.specifiedArrivalTime = !arrivalTime.IsNull();
    timeToSearchRoute.specifiedDepartureTime = !departureTime.IsNull();

    timeToSearchRoute.departureTime = earlyStartTime;
    timeToSearchRoute.earlyDepartureTime = earlyStartTime;
    timeToSearchRoute.lateDepartureTime = INFINITE_TIME;

    if (!timeToSearchRoute.specifiedArrivalTime &&
        !timeToSearchRoute.specifiedDepartureTime) {

        timeToSearchRoute.earlyArrivalTime = ZERO_TIME;
        timeToSearchRoute.arrivalTime = INFINITE_TIME;
        timeToSearchRoute.lateArrivalTime = INFINITE_TIME;
        return;
    }

    if (!arrivalTime.IsNull()) {
        timeToSearchRoute.arrivalTime =
            std::max(earlyStartTime, arrivalTime.CalculateTime(resource));
    }
    if (!earlyArrivalTime.IsNull()) {
        timeToSearchRoute.earlyArrivalTime =
            std::max(earlyStartTime, earlyArrivalTime.CalculateTime(resource));
    }
    if (!lateArrivalTime.IsNull()) {
        timeToSearchRoute.lateArrivalTime =
            std::max(earlyStartTime, lateArrivalTime.CalculateTime(resource));
    }
    if (!departureTime.IsNull()) {
        timeToSearchRoute.departureTime =
            std::max(earlyStartTime, departureTime.CalculateTime(resource));
    }
    if (!earlyDepartureTime.IsNull()) {
        timeToSearchRoute.earlyDepartureTime =
            std::max(earlyStartTime, earlyDepartureTime.CalculateTime(resource));
    }
    if (!lateDepartureTime.IsNull()) {
        timeToSearchRoute.lateDepartureTime =
            std::max(earlyStartTime, lateDepartureTime.CalculateTime(resource));
    }

    timeToSearchRoute.earlyArrivalTime =
        std::min(timeToSearchRoute.earlyArrivalTime, timeToSearchRoute.arrivalTime);

    timeToSearchRoute.lateArrivalTime =
        std::max(timeToSearchRoute.lateArrivalTime, timeToSearchRoute.arrivalTime);

    timeToSearchRoute.earlyDepartureTime =
        std::min(timeToSearchRoute.earlyDepartureTime, timeToSearchRoute.departureTime);

    timeToSearchRoute.lateDepartureTime =
        std::max(timeToSearchRoute.lateDepartureTime, timeToSearchRoute.departureTime);
}

void AgentTaskTable::GetStatusChanges(
    const AgentResource& resource,
    priority_queue_stable<AgentStatusChangeEvent>& timeLineStatusChangeEvents) const
{
    while (!timeLineStatusChangeEvents.empty()) {
        timeLineStatusChangeEvents.pop();
    }

    for(size_t i = 0; i < statusChangePtrs.size(); i++) {
        const AgentTask& statusChangeTask = *(statusChangePtrs[i]);
        const TimeType changeTime = statusChangeTask.GetStartTime(resource);

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                changeTime,
                i,
                AGENT_STATUS_CHANGE_AT_SPECIFIC_TIME));
    }

    for(size_t i = 0; i < interruptTaskPtrs.size(); i++) {
        const AgentTask& statusChangeTask = *(interruptTaskPtrs[i]);
        const TimeType changeTime = statusChangeTask.GetStartTime(resource);

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                changeTime,
                i,
                AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START));
    }
}

//----------------------------------------------------------------------------------------

AgentConditionChecker::AgentConditionChecker(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& initParameters,
    const ConditionParameterType& initParameterType,
    const double initSimStartTimeSec,
    const string& initString,
    map<string, AgentCharactorIdType>& initCharactorIds)
    :
    parameterType(initParameterType)
{
    assert(initString.size() > 2);

    const string checkString = initString.substr(0, 2);
    size_t checkStringLength = 2;

    if (checkString == "==") {
        Checker = &AgentConditionChecker::Check1;
    } else if (checkString == "!=") {
        Checker = &AgentConditionChecker::Check2;
    } else if (checkString == "<=") {
        Checker = &AgentConditionChecker::Check4;
    } else if (checkString == ">=") {
        Checker = &AgentConditionChecker::Check6;
    } else if (checkString[0] == '<') {
        Checker = &AgentConditionChecker::Check3;
        checkStringLength = 1;
    } else if (checkString[0] == '>') {
        Checker = &AgentConditionChecker::Check5;
        checkStringLength = 1;
    } else {
        Checker = &AgentConditionChecker::Check1;
        checkStringLength = 0;
    }

    formula = AgentValueFormula(
        initParameters,
        initSimStartTimeSec,
        initString.substr(checkStringLength),
        initCharactorIds);
}

bool AgentConditionChecker::Check(const AgentResource& resource) const
{
    if (parameterType == CONDITION_PARAMETER_TIME) {
        return (this->*(Checker))(double(resource.CurrentTime()/SECOND), formula.CalculateDouble(resource));
    } else {
        return (this->*(Checker))(resource.Value(ConvertToStatusId(parameterType)), formula.CalculateDouble(resource));
    }
}

AgentProfile::AgentProfile(
    const AgentProfileType& initProfileType,
    const string& initProfileName)
    :
    profileName(initProfileName),
    profileType(initProfileType),
    userType(AGENT_USER_TYPE_NONE),
    mobilityClass(AGENT_MOBILTY_CLASS_NORMAL),
    ticketType(AGENT_TICKET_FULL_FARE),
    routeCostFormulas(NUMBER_AGENT_BEHAVIORS)
{
    for(AgentStatusIdType statusId = 0; statusId < NUMBER_AGENT_STATUS_VALUES; statusId++) {

        double defaultValue = 0;

        if (AGENT_RESERVED_STATUS_QUERY_TRIGGER_START <= statusId &&
            statusId <= AGENT_RESERVED_STATUS_QUERY_TRIGGER_END) {
            defaultValue = DBL_MAX;
        }
        parameters[RESERVED_AGENT_STATUS_NAMES[statusId]] = AgentValueFormula(defaultValue);
    }

    parameters[AGENT_RESERVED_STATUS_SEEING_PEDESTRIAN_PROBABILITY] = DEFAULT_SEEING_PEDESTRIAN_PROBABILITY;
    parameters[AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED] = DEFAULT_MAX_VEHICLE_SPEED;
    parameters[AGENT_RESERVED_LANE_CHANGE_ACCELERATION_THRESHOLD] = DEFAULT_LANE_CHANGE_ACCELERATION_THRESHOLD;
    parameters[AGENT_RESERVED_STATUS_TIME_HEADWAY] = DEFAULT_TIME_HEADWAY;
    parameters[AGENT_RESERVED_STATUS_MIN_VEHICLE_GAP] = DEFAULT_MIN_VEHICLE_GAP;
    parameters[AGENT_RESERVED_STATUS_MAX_ACCELERATION] = DEFAULT_MAX_ACCELERATION;
    parameters[AGENT_RESERVED_STATUS_MAX_DECELERATION] = DEFAULT_MAX_DECELERATION;
    parameters[AGENT_RESERVED_VELOCITY_RATIO_GAP_DISTANCE] = DEFAULT_VELOCITY_RATIO_GAP_DISTANCE;
    parameters[AGENT_RESERVED_OTHER_VEHICLE_ENATRANCE_TIME] = DEFAULT_OTHER_VEHICLE_ENATRANCE_TIME;
    parameters[AGENT_RESERVED_PASSIVE_YIELD_TIME] = DEFAULT_PASSIVE_YIELD_TIME;
    parameters[AGENT_RESERVED_ACTIVE_YIELD_TIME] = DEFAULT_ACTIVE_YIELD_TIME;
    parameters[AGENT_RESERVED_YIELD_WAITING_TIME] = DEFAULT_YIELD_WAITING_TIME;
    parameters[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_CAR] = DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_VEHICLE;
    parameters[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_STOP] = DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_STOP;
    parameters[AGENT_RESERVED_MIN_VEHICLE_ROUTE_DISTANCE] = DEFAULT_MIN_VEHICLE_ROUTE_DISTANCE;
    parameters[AGENT_RESERVED_NUMBER_MAX_ROUTE_CANDIDATES] = DEFAULT_NUMBER_MAX_ROUTE_CANDIDATES;
    parameters[AGENT_RESERVED_NUMBER_PEOPLE] = DEFAULT_NUMBER_PEOPLE;
    parameters[AGENT_RESERVED_ENTRANCE_WAIT_TIME] = DEFAULT_ENTRANCE_WAIT_TIME;
    parameters[AGENT_RESERVED_TAXICALL_WAIT_TIME] = DEFAULT_TAXICALL_WAIT_TIME;
    parameters[AGENT_RESERVED_STATUS_MAX_BRAKING_DECCELERATION] = DEFAULT_MAX_BRAKING_DECCELERATION;
    parameters[AGENT_RESERVED_STATUS_ACCELERATION_EXPONENT] = DEFAULT_ACCELERATION_EXPONENT;
    parameters[AGENT_RESERVED_STATUS_SAVE_DECELERATION] = DEFAULT_SAVE_DECELERATION;
    parameters[AGENT_RESERVED_STATUS_MAX_TURN_SPEED] = DEFAULT_MAX_TURN_SPEED;
}

static inline
void MakeSetOfAllAgentIds(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    set<AgentIdType>& agentIds)
{
    set<AgentIdType> profileAgentIds;
    set<AgentIdType> behaviorAgentIds;

    theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter("multiagent-profile-type", profileAgentIds);
    theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter("multiagent-behavior-type", behaviorAgentIds);

    agentIds.insert(profileAgentIds.begin(), profileAgentIds.end());
    agentIds.insert(behaviorAgentIds.begin(), behaviorAgentIds.end());
}

//--------------------------------------------------------------------

const string AGENT_TYPE_TAXI("taxi");
const string AGENT_TYPE_BUS("bus");

const string MultiAgentSimulator::modelName = "Mas";

#pragma warning(disable:4355)

MultiAgentSimulator::MultiAgentSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially),
    currentSnapshotId(0),
    currentTime(ZERO_TIME),
    timeStep(
        theParameterDatabaseReaderPtr->ReadTime(
            "time-step-event-synchronization-step")),
    isSimulationDone(false),
    numberThreads(
        std::max(1, theParameterDatabaseReaderPtr->ReadInt(
                     "number-data-parallel-threads-for-multiagent"))),
    profileValueOutputSubsystem(*theParameterDatabaseReaderPtr),
    threadPartitions(numberThreads),
    timeIncrementThreadBarrier(numberThreads),
    reservedVehicleNodePtrs(NUMBER_VEHICLE_TYPES),
    theAgentGisPtr(
        new MultiAgentGis(
            this,
            *theParameterDatabaseReaderPtr,
            initSimulationEnginePtr,
            theGisSubsystemPtr,
            numberThreads)),
    masterAnyAgentPtr(
        Agent::CreateMasterAgent(
            this,
            MASTER_ANY_AGENT_ID,
            shared_ptr<AgentProfile>(new AgentProfile(INVALID_AGENT_TYPE)),
            shared_ptr<AgentTaskTable>(new AgentTaskTable(nullptr)))),
    thePublicVehicleTablePtr(
        new PublicVehicleTable(
            this,
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            numberThreads)),
    theProfileAndTaskTable(
        *theParameterDatabaseReaderPtr,
        thePublicVehicleTablePtr,
        theGisSubsystemPtr,
        masterAnyAgentPtr,
        numberThreads),
    masterBusAgentPtr(
        Agent::CreateMasterAgent(
            this,
            MASTER_BUS_AGENT_ID,
            theProfileAndTaskTable.GetProfile(AGENT_TYPE_BUS),
            theProfileAndTaskTable.GetEmptyTaskTable())),
    masterTaxiAgentPtr(
        Agent::CreateMasterAgent(
            this,
            MASTER_TAXI_AGENT_ID,
            theProfileAndTaskTable.GetProfile(AGENT_TYPE_TAXI),
            theProfileAndTaskTable.GetEmptyTaskTable())),
    theRouteSearchSubsystemPtr(
        new AgentRouteSearchSubsystem(
            theAgentGisPtr,
            thePublicVehicleTablePtr)),
    theSimulationRunInterfacePtr(
        theSimulationEnginePtr->GetSimulationEngineInterface(
            *initParameterDatabaseReaderPtr, ScenSim::ANY_NODEID))
{
    const ParameterDatabaseReader& theParameterDatabaseReader = *initParameterDatabaseReaderPtr;

    thePublicVehicleTablePtr->CompleteRouteAndVehiclewScheduleInitialization();

    if (!theParameterDatabaseReader.ReadBool("gis-road-set-intersection-margin")) {
        cerr << "Error in MultiAgent Extension Module: Set gis-road-set-intersection-margin = true" << endl;
        exit(1);
    }

    if (numberThreads > NUMBER_MAX_THREADS) {
        cerr << "Error: Number Max Threads is " << NUMBER_MAX_THREADS << endl;
        exit(1);
    }

    MakeSetOfAllAgentIds(*theParameterDatabaseReaderPtr, entireAgentIds);

    theProfileAndTaskTable.CompleteInitialize(
        *theParameterDatabaseReaderPtr,
        theGisSubsystemPtr,
        masterAnyAgentPtr,
        entireAgentIds);

    if (timeStep < MULTIAGENT_MIN_TIME_STEP) {
        cerr << "Error:  Multi-Agent min time step is 1 ms" << endl;
        exit(1);
    }

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        ThreadPartition& threadPartition = threadPartitions[i];

        if (i != MASTER_THREAD_NUMBER) {
            threadPartition.timeIncrementThreadPtr.reset(
                new boost::thread(TimeIncrementThreadFunctor(this, i)));
        }
    }

    set<AgentIdType> agentIds;
    vector<shared_ptr<BusTicket> > busReservationPtrs;

    MakeSetOfAllAgentIds(*theParameterDatabaseReaderPtr, agentIds);

    typedef set<NodeIdType>::const_iterator IterType;

    map<AgentIdType, Vertex> taxiAgentLocations;

    for(IterType iter = agentIds.begin(); iter != agentIds.end(); iter++) {
        const AgentIdType& agentId = (*iter);

        string profileName = MakeLowerCaseString(
            theParameterDatabaseReaderPtr->ReadString(
                "multiagent-profile-type", agentId));

        string taskName = MakeLowerCaseString(
            theParameterDatabaseReaderPtr->ReadString(
                "multiagent-behavior-type", agentId));

        if (AStringStartsWith(profileName, "taxi")) {

            const double distanceGranularityMeters = 0;
            const string positionFileName =
                theParameterDatabaseReaderPtr->ReadString(
                    "mobility-init-positions-file",
                    agentId);

            InorderFileCache mobilityFileCache;

            TraceFileMobilityModel traceMobilityForTaxi(
                *theParameterDatabaseReaderPtr,
                agentId,
                nullInstanceId,
                mobilityFileCache,
                positionFileName,
                agentId,
                distanceGranularityMeters,
                theGisSubsystemPtr);

            ObjectMobilityPosition taxiMobilityPosition;

            traceMobilityForTaxi.GetUnadjustedPositionForTime(ZERO_TIME, taxiMobilityPosition);

            const Vertex taxiPos(
                taxiMobilityPosition.X_PositionMeters(),
                taxiMobilityPosition.Y_PositionMeters());

            taxiAgentLocations[agentId] = taxiPos;

        } else if (profileName == "train" ||
                   profileName == "bus" ||
                   profileName == "vehicle" ||
                   profileName == "privatecar") {

            VehicleType vehicleType = VEHICLE_CAR;

            if (profileName == "train") {
                vehicleType = VEHICLE_TRAIN;
            } else if (profileName == "bus") {
                vehicleType = VEHICLE_BUS;
            } else if (profileName == "vehicle" || profileName == "privatecar") {
                vehicleType = VEHICLE_CAR;
            }

            (*this).ReserveVehicleNode(agentId, vehicleType);

        } else {

            size_t idExchangePos = profileName.find("$n");

            if (idExchangePos != string::npos) {
                profileName =
                    profileName.substr(0, idExchangePos) +
                    ConvertToString(agentId) +
                    profileName.substr(idExchangePos + 2);
            }

            const shared_ptr<Agent> agentPtr(
                new Agent(
                    this,
                    theGlobalNetworkingObjectBag,
                    agentId,
                    (*this).GetSimEngineInterfacePtr(agentId),
                    theProfileAndTaskTable.GetProfile(profileName),
                    theProfileAndTaskTable.GetTaskTable(taskName, profileName),
                    theAgentGisPtr,
                    thePublicVehicleTablePtr,
                    theRouteSearchSubsystemPtr));
            if(debugMode == true){
                cout << "MultiAgentSimulator__new AgentId:" << agentId << endl;
            }
            if (agentPtr->HasCar()) {
                threadPartitions[MASTER_THREAD_NUMBER].
                    newlyAddedVehiclePtrs.push_back(agentPtr->GetVehicle());
            }

            const AgentResource resource(agentPtr);
            const double lastDelayQueryTrigger = resource.LastDelayQueryTrigger();
            const double nextDelayQueryTrigger = resource.NextDelayQueryTrigger();
            const double tripDelayQueryTrigger = resource.TripDelayQueryTrigger();
            const double vehicleDelayQueryTrigger = resource.VehicleDelayQueryTrigger();
            const double congestionQueryTrigger = resource.CongestionQueryTrigger();
            const double utility1QueryTrigger = resource.Utility1QueryTrigger();
            const double utility2QueryTrigger = resource.Utility2QueryTrigger();

            if (lastDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForLastViaPointDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (nextDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForNextViaPointDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (tripDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForDestinationDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (vehicleDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForVehicleDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (congestionQueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForCongestion to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (utility1QueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForUtility1 to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (utility2QueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForUtility2 to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }

            const TimeType wakeupTime = agentPtr->CalculateWakeupTime();

            agentWakeupQueue.push(AgentWakeupEntry(wakeupTime, agentPtr));

            wakeupTimes[agentId] = wakeupTime;
        }
    }

    //theGisSubsystemPtr->AddGisChangeEventHandler(
    //    modelName,
    //    shared_ptr<MultiAgentGisChangeEventHandler>( new MultiAgentGisChangeEventHandler(this)));

    thePublicVehicleTablePtr->CompleteAllPublicVehicleInitialization();

    theAgentGisPtr->CompleteInitialization(
        *theParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        *thePublicVehicleTablePtr,
        theProfileAndTaskTable,
        taxiAgentLocations);

    // synchronizer -----------------------------------------

    preSynchronizerPtrs.push(theAgentGisPtr->CreateVeicleAgentSynchronizer());
    preSynchronizerPtrs.push(theAgentGisPtr->CreatePublicVehicleSynchronizer(thePublicVehicleTablePtr));
    preSynchronizerPtrs.push(theAgentGisPtr->CreateTaxiSynchronizer());
    preSynchronizerPtrs.push(theAgentGisPtr->CreateCrossingPedestrianSynchronizer());
    preSynchronizerPtrs.push(theAgentGisPtr->CreateTopologySynchronizer());

    // Application constructor must not output trace because of trace output confliction between threads.
    // If an application output trace in constructor, call SyncApplicationCreation() without ApplicationCreationSynchronizer

    preSynchronizerPtrs.push(shared_ptr<ApplicationCreationSynchronizer>(new ApplicationCreationSynchronizer(this)));



    (*this).SetupStatOutputFile();

}//MultiAgentSimulator()//

#pragma warning(default:4355)

MultiAgentSimulator::~MultiAgentSimulator()
{
    isSimulationDone = true;
    timeIncrementThreadBarrier.wait();

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        if (i != MASTER_THREAD_NUMBER) {
            threadPartitions[i].timeIncrementThreadPtr->join();
        }
    }

    theAgentGisPtr->DisconnectGisConnection();

    (*this).DeleteAllNodes();

    while (!preSynchronizerPtrs.empty()) {
        preSynchronizerPtrs.pop();
    }

    reservedVehicleNodePtrs.clear();
    theAgentGisPtr.reset();
    masterAnyAgentPtr.reset();
    thePublicVehicleTablePtr.reset();

    masterBusAgentPtr.reset();
    masterTaxiAgentPtr.reset();

    theRouteSearchSubsystemPtr.reset();
    theSimulationRunInterfacePtr.reset();

    entireAgentIds.clear();

    while(!agentWakeupQueue.empty()) {
        agentWakeupQueue.pop();
    }

    newlyAddedAgentIds.clear();

    communicationNodePtrsWaitingAgentCreation.clear();
    synchronizedNodePtrs.clear();
    agentThreadNumbers.clear();

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        ThreadPartition& threadPartition = threadPartitions[i];

        threadPartition.timeIncrementThreadPtr.reset();

        while(!threadPartition.deleteNodeIds.empty()) {
            threadPartition.deleteNodeIds.pop();
        }

        threadPartition.agentPtrs.clear();

        while (!threadPartition.tookSynchronizerPtrs.empty()) {
            threadPartition.tookSynchronizerPtrs.pop();
        }

        threadPartition.iterToAgentList.clear();
        threadPartition.newlyAddedVehiclePtrs.clear();
        threadPartition.ownerChangeEvents.clear();
    }

    threadPartitions.clear();
}

MultiAgentSimulator::ProfileValueOutputSubsystem::ProfileValueOutputSubsystem(
    const ParameterDatabaseReader& initParameterDatabaseReader)
{
    if (initParameterDatabaseReader.ParameterExists("multiagent-profile-value-output-file")) {
        const string profileValueOutputFileName =
            initParameterDatabaseReader.ReadString("multiagent-profile-value-output-file");

        const string agentProfileFilePath =
            initParameterDatabaseReader.ReadString("multiagent-profile-file");

        if (agentProfileFilePath == profileValueOutputFileName) {
            cerr << "Error: Set \"multiagent-profile-value-output-file\" value other than \"multiagent-profile-file\" value." << endl;
            exit(1);
        }

        profileValueOutputFile.open(profileValueOutputFileName.c_str());

        if (!profileValueOutputFile.is_open()) {
            cerr << "Error: Could not open profile value output file " << profileValueOutputFileName << endl;
            exit(1);
        }
    }
}

MultiAgentSimulator::ThreadPartition::~ThreadPartition()
{
}

void MultiAgentSimulator::RunSimulationUntil(const TimeType& snapshotTime)
{
    while (currentTime <= snapshotTime) {
        theSimulationRunInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new AdvanceTimeStepEvent(this)), currentTime);

        theSimulationEnginePtr->RunSimulationSequentially(currentTime);
    }
}

void MultiAgentSimulator::AdvanceTimeStep()
{
    if (isDebugMode) {
        cout << "SimTime = " << ConvertTimeToDoubleSecs(currentTime) << "[s]" << endl;
    }

    newlyAddedAgentIds.clear();

    (*this).AddVehicle();

    while (!agentWakeupQueue.empty() &&
           agentWakeupQueue.top().time <= currentTime) {

        (*this).AddAgent(agentWakeupQueue.top().agentPtr);

        agentWakeupQueue.pop();
    }

    (*this).ChangeAgentOwnerIfNecessary();

    (*this).IncrementTimeStep();

    (*this).DeleteInactiveAgents();

    currentTime += timeStep;
}

void MultiAgentSimulator::AddVehicle()
{
    vector<shared_ptr<Train> > trainPtrs;
    vector<shared_ptr<Bus> > busPtrs;

    thePublicVehicleTablePtr->CreatePublicVehicles(currentTime, trainPtrs, busPtrs);

    (*this).AddTrain(trainPtrs);

    (*this).AddBus(busPtrs);

    (*this).AddCar();
}

void MultiAgentSimulator::AddTrain(const vector<shared_ptr<Train> >& trainPtrs)
{
    if (trainPtrs.empty()) {
        return;
    }

    queue<shared_ptr<VehicleNode> >& trainNodePtrs = reservedVehicleNodePtrs.at(VEHICLE_TRAIN);

    for(size_t i = 0; i < trainPtrs.size(); i++) {
        const shared_ptr<Train>& trainPtr = trainPtrs[i];

        if (trainNodePtrs.size() < trainPtrs.size()) {
            cerr << "Error: not enough train node" << endl;
            exit(1);
        }

        assert(!trainNodePtrs.empty());
        const shared_ptr<VehicleNode> trainNodePtr = trainNodePtrs.front();
        const AgentIdType agentId = trainNodePtr->GetAgentId();

        shared_ptr<Agent> trainDriverPtr = Agent::CreateTrainDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            theProfileAndTaskTable.GetProfile(AGENT_TYPE_BUS),
            theProfileAndTaskTable.GetEmptyTaskTable(),
            theAgentGisPtr,
            thePublicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            trainNodePtr,
            trainPtr);

        trainPtr->SetDriverAgentId(agentId);

        (*this).AddNode(trainNodePtr);
        (*this).AddAgentWithoutNodeGeneration(trainDriverPtr);

        trainNodePtr->SetPublicVehicle(
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            (*this).CurrentTime(),
            trainPtr);

        theAgentGisPtr->AddTrain(trainPtr);

        trainNodePtrs.pop();

        (*this).OutputTrace("Create Train " + ConvertToString(trainNodePtr->GetAgentId()));
    }
}

void MultiAgentSimulator::AddBus(const vector<shared_ptr<Bus> >& busPtrs)
{
    if (busPtrs.empty()) {
        return;
    }


    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    for(size_t i = 0; i < busPtrs.size(); i++) {
        const shared_ptr<Bus>& busPtr = busPtrs[i];

        queue<shared_ptr<VehicleNode> >& busNodePtrs =
            reservedVehicleNodePtrs.at(busPtr->GetVehicleType());

        if (busNodePtrs.empty()) {
            cerr << "Error: not enough bus node" << endl;
            exit(1);
        }

        const shared_ptr<VehicleNode> busNodePtr = busNodePtrs.front();
        const AgentIdType agentId = busNodePtr->GetAgentId();
        const RoadIdType roadId = busPtr->GetStartRoadId();
        const BusStopIdType startBusStopId = busPtr->GetStartBusStopId();
        const VertexIdType vertexId = subsystem.GetBusStop(startBusStopId).GetVertexId();
        const size_t laneNumber = subsystem.GetRoad(roadId).GetOutsideOutgoingLaneNumber(vertexId);

        shared_ptr<Vehicle> vehiclePtr(
            new Vehicle(
                agentId,
                vertexId,
                theAgentGisPtr->GetVertex(vertexId),
                roadId,
                laneNumber,
                this));

        shared_ptr<Agent> busDriverPtr = Agent::CreateBusDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            theProfileAndTaskTable.GetProfile(AGENT_TYPE_BUS),
            theProfileAndTaskTable.GetEmptyTaskTable(),
            theAgentGisPtr,
            thePublicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            vehiclePtr,
            busNodePtr,
            busPtr);

        (*this).AddNode(busNodePtr);
        (*this).AddAgentWithoutNodeGeneration(busDriverPtr);

        busPtr->SetVehicle(vehiclePtr);

        busNodePtr->SetPublicVehicle(
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            (*this).CurrentTime(),
            busPtr);

        theAgentGisPtr->AddVehicle(vehiclePtr);

        busNodePtrs.pop();

        (*this).OutputTrace("Create Bus " + ConvertToString(agentId));
    }
}

void MultiAgentSimulator::ProfileValueOutputSubsystem::RecordAssignedProfileValuesToFile(
    const AgentIdType& agentId,
    const shared_ptr<AgentProfile>& profilePtr,
    const map<string, double>& assignedValues,
    const bool hasCar,
    const bool hasBicycle)
{
    if (!profileValueOutputFile.is_open()) {
        return;
    }

    if (agentId == MASTER_ANY_AGENT_ID) {
        // Skip master agent profile
        return;
    }

    typedef map<string, double>::const_iterator IterType;

    if (agentId == MASTER_BUS_AGENT_ID) {

        profileValueOutputFile << "ProfileType:Bus" << endl;

    } else if (agentId == MASTER_TAXI_AGENT_ID) {

        profileValueOutputFile << "ProfileType:Taxi" << endl;

    } else {
        profileValueOutputFile << "ProfileType:Agent" << agentId << "_" << profilePtr->GetProfileName() << endl;
    }

    for(IterType iter = assignedValues.begin();
        iter != assignedValues.end(); iter++) {

        const string& profileName = (*iter).first;
        const double value = (*iter).second;

        profileValueOutputFile << profileName << " = " << value << endl;
    }

    if ((agentId != MASTER_BUS_AGENT_ID) &&
        (agentId != MASTER_TAXI_AGENT_ID)) {

        double privateCarOwnership = 0.0;

        if (hasCar) {
            privateCarOwnership = 1.0;
        }

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_PRIVATE_CAR_OWNERSHIP] << " = " << privateCarOwnership << endl;

        double bicycleOwnership = 0.0;

        if (hasBicycle) {
            bicycleOwnership = 1.0;
        }

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_BICYCLE_OWNERSHIP] << " = " << bicycleOwnership << endl;

        const AgentMobilityClassType mobilityClass = profilePtr->GetMobilityClass();

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_WALK_SPEED_AT_TRANSFER] << " = " << GetAgentMobilityClassName(mobilityClass) << endl;

        const AgentValueFormula& utilityFormula1 = profilePtr->GetUtilityFormula1();
        const AgentValueFormula& utilityFormula2 = profilePtr->GetUtilityFormula2();

        if (!utilityFormula1.IsNull()) {
            profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY1_FUNCTION] << " = " << utilityFormula1.GetInputFormulaString() << endl;
        }
        if (!utilityFormula2.IsNull()) {
            profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY2_FUNCTION] << " = " << utilityFormula2.GetInputFormulaString() << endl;
        }
    }

    profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_ROUTE_PRIORITY] << " = " << profilePtr->GetPrimaryRouteCostFormula().GetInputFormulaString() << endl;

    profileValueOutputFile << endl;
}

void MultiAgentSimulator::AddCar()
{
    queue<shared_ptr<VehicleNode> >& vehicleNodePtrs = reservedVehicleNodePtrs.at(VEHICLE_CAR);

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        list<shared_ptr<Vehicle> >& vehiclePtrs = threadPartitions[i].newlyAddedVehiclePtrs;

        typedef list<shared_ptr<Vehicle> >::const_iterator IterType;

        for(IterType iter = vehiclePtrs.begin(); iter != vehiclePtrs.end(); iter++) {
            const shared_ptr<Vehicle>& vehiclePtr = (*iter);

            if (vehicleNodePtrs.empty()) {
                cerr << "Error: not enough vehicle node" << endl;
                exit(1);
            }

            shared_ptr<VehicleNode> vehicleNodePtr = vehicleNodePtrs.front();

            vehicleNodePtr->SetVehicle(
                *theParameterDatabaseReaderPtr,
                theAgentGisPtr,
                (*this).CurrentTime(),
                vehiclePtr);

            (*this).AddNode(vehicleNodePtr);

            vehicleNodePtrs.pop();

            theAgentGisPtr->AddVehicle(vehiclePtr);
        }
        vehiclePtrs.clear();
    }
}

void MultiAgentSimulator::AddTaxi(const shared_ptr<Taxi>& taxiPtr)
{
    const AgentIdType agentId = taxiPtr->GetDriverAgentId();

    const shared_ptr<VehicleNode> vehicleNodePtr =
        (*this).CreateVehicleNode(agentId, VEHICLE_TAXI);

    const shared_ptr<Agent> taxiDriverPtr =
        Agent::CreateTaxiDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            theProfileAndTaskTable.GetProfile(AGENT_TYPE_TAXI),
            theProfileAndTaskTable.GetEmptyTaskTable(),
            theAgentGisPtr,
            thePublicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            vehicleNodePtr,
            taxiPtr);

    (*this).AddNode(vehicleNodePtr);
    (*this).AddAgentWithoutNodeGeneration(taxiDriverPtr);

    vehicleNodePtr->SetTaxi(
        *theParameterDatabaseReaderPtr,
        theAgentGisPtr,
        (*this).CurrentTime(),
        taxiPtr);

    theAgentGisPtr->AddVehicle(taxiPtr);
}


void MultiAgentSimulator::AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& aNodePtr)
{
    const AgentIdType agentId = aNodePtr->GetNodeId();

    if (entireAgentIds.find(agentId) == entireAgentIds.end()) {

        (*this).AddNode(aNodePtr);

    } else {

        synchronizedNodePtrs[agentId] = aNodePtr;

        (*this).AttachCommunicationNode(agentId, aNodePtr);
    }
}

bool MultiAgentSimulator::IsEqualToAgentId(const NodeIdType& nodeId) const
{
    return (entireAgentIds.find(nodeId) != entireAgentIds.end());
}

TimeType MultiAgentSimulator::GetWakeupTime(const NodeIdType& nodeId) const
{
    assert((*this).IsEqualToAgentId(nodeId));

    typedef map<NodeIdType, TimeType>::const_iterator IterType;

    IterType iter = wakeupTimes.find(nodeId);

    assert(iter != wakeupTimes.end());

    return (*iter).second;
}

void MultiAgentSimulator::AddAgent(
    const shared_ptr<Agent>& agentPtr,
    const bool withNodeGeneration)
{
    AgentResource resource(agentPtr);
    const AgentIdType agentId = resource.AgentId();

    // multiagent
    const size_t threadNumber = agentId % threadPartitions.size();

    ThreadPartition& threadPartition = threadPartitions.at(threadNumber);
    list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;
    map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList =
        threadPartition.iterToAgentList;

    iterToAgentList[agentId] = agentPtrs.insert(agentPtrs.end(), agentPtr);

    newlyAddedAgentIds.insert(agentId);

    agentThreadNumbers[agentId] = threadNumber;

    // sim
    if (withNodeGeneration) {
        shared_ptr<BasicNetworkNode> simNodePtr;

        if (createCommunicationNodeAtWakeupTimeFor.find(agentId) != createCommunicationNodeAtWakeupTimeFor.end()) {
            (*this).CreateNewNode(*theParameterDatabaseReaderPtr, agentId, agentPtr->GetMobilityModelPtr());
        }

        if (synchronizedNodePtrs.find(agentId) != synchronizedNodePtrs.end()) {

            simNodePtr = synchronizedNodePtrs[agentId];
            synchronizedNodePtrs.erase(agentId);

        } else {
            simNodePtr.reset(
                new AgentNode(
                    *theParameterDatabaseReaderPtr,
                    theGlobalNetworkingObjectBag,
                    theGisSubsystemPtr,
                    agentPtr->GetSimEngineInterfacePtr(),
                    agentId,
                    runSeed,
                    agentPtr->GetMobilityModelPtr(),
                    resource));
        }

        (*this).AddNode(simNodePtr);
    }

    theAgentGisPtr->UpdatePeopleTranslationBetweenGisObjects(
        resource,
        GisPositionIdType(),
        resource.PositionId(),
        AGENT_BEHAVIOR_NOTHING,
        AGENT_BEHAVIOR_NOTHING);

    resource.ArrivedAtGisPositionNotification();

    typedef map<AgentIdType, list<shared_ptr<AgentCommunicationNode> > >::iterator CommunicationNodesIter;

    CommunicationNodesIter communicationNodesIter = communicationNodePtrsWaitingAgentCreation.find(agentId);

    if (communicationNodesIter != communicationNodePtrsWaitingAgentCreation.end()) {
        list<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = (*communicationNodesIter).second;

        typedef list<shared_ptr<AgentCommunicationNode> >::iterator NodeIter;

        for(NodeIter nodeIter = communicationNodePtrs.begin();
            nodeIter != communicationNodePtrs.end(); nodeIter++) {

            agentPtr->AddCommunicationNode(*nodeIter);
        }

        communicationNodePtrsWaitingAgentCreation.erase(agentId);
    }


    (*this).OutputTrace("Add Agent " + ConvertToString(agentId));
}

shared_ptr<SimulationEngineInterface> MultiAgentSimulator::GetSimEngineInterfacePtr(const AgentIdType& agentId)
{
    unsigned int partitionIndex = 0;
    if (theParameterDatabaseReaderPtr->ParameterExists(
            "parallelization-partition-index", agentId)) {
        partitionIndex =
            (theParameterDatabaseReaderPtr->ReadNonNegativeInt(
                "parallelization-partition-index", agentId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }

    return theSimulationEnginePtr->GetSimulationEngineInterface(
        *theParameterDatabaseReaderPtr, agentId, partitionIndex);
}

shared_ptr<VehicleNode> MultiAgentSimulator::CreateVehicleNode(
    const AgentIdType& agentId,
    const VehicleType& vehicleType)
{
    unsigned int partitionIndex = 0;

    if (theParameterDatabaseReaderPtr->ParameterExists(
            "parallelization-partition-index", agentId)) {
        partitionIndex =
            (theParameterDatabaseReaderPtr->ReadNonNegativeInt(
                "parallelization-partition-index", agentId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }

    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            *theParameterDatabaseReaderPtr, agentId, partitionIndex);

    shared_ptr<VehicleMobilityModel> mobilityModelPtr(new VehicleMobilityModel(agentId));

    shared_ptr<VehicleNode> vehicleNodePtr(
        new VehicleNode(
            *theParameterDatabaseReaderPtr,
            theGlobalNetworkingObjectBag,
            theGisSubsystemPtr,
            simEngineInterfacePtr,
            agentId,
            runSeed,
            mobilityModelPtr));

    return vehicleNodePtr;
}

void MultiAgentSimulator::ReserveVehicleNode(
    const AgentIdType& agentId,
    const VehicleType& vehicleType)
{
    const shared_ptr<VehicleNode> vehicleNodePtr =
        (*this).CreateVehicleNode(agentId, vehicleType);

    reservedVehicleNodePtrs.at(vehicleType).push(vehicleNodePtr);
}

void MultiAgentSimulator::SetOwnerAgent(
    const AgentResource& resource,
    const AgentIdType& ownerAgentId)
{
    if (threadPartitions.empty()) {
        // simulation is done.
        return;
    }

    ThreadPartition& threadPartition = threadPartitions.at(resource.ThreadNumber());

    assert(resource.AgentId() != ownerAgentId);

    threadPartition.ownerChangeEvents[resource.AgentId()] =
        OwnerChangeEvent(resource.AgentId(), ownerAgentId);
}

void MultiAgentSimulator::RemoveOwnerAgentChange(const AgentResource& resource)
{
    if (threadPartitions.empty()) {
        // simulation is done.
        return;
    }

    ThreadPartition& threadPartition = threadPartitions.at(resource.ThreadNumber());

    threadPartition.ownerChangeEvents.erase(resource.AgentId());
}

void MultiAgentSimulator::ChangeAgentOwnerIfNecessary()
{
    list<pair<AgentIdType, shared_ptr<Agent> > > freeAgentPtrs;

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        ThreadPartition& threadPartition = threadPartitions[i];
        map<AgentIdType, OwnerChangeEvent>& ownerChangeEvents = threadPartition.ownerChangeEvents;
        list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;
        map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

        typedef map<AgentIdType, OwnerChangeEvent>::const_iterator IterType;

        for(IterType iter = ownerChangeEvents.begin();
            iter != ownerChangeEvents.end(); iter++) {

            const OwnerChangeEvent& ownerChangeEvent = (*iter).second;

            assert(iterToAgentList.find(ownerChangeEvent.agentId) != iterToAgentList.end());

            typedef list<shared_ptr<Agent> >::iterator AgentIter;

            AgentIter childIter = iterToAgentList[ownerChangeEvent.agentId];

            const shared_ptr<Agent> childAgentPtr = (*childIter);

            if (ownerChangeEvent.ownerId == MASTER_ANY_AGENT_ID) {
                assert(childAgentPtr->HasParent());

                AgentIter parentIter = iterToAgentList[childAgentPtr->parentAgentId];
                const shared_ptr<Agent> parentAgentPtr = (*parentIter);

                parentAgentPtr->childAgentPtrs.erase(childIter);

            } else {
                assert(!childAgentPtr->HasParent());

                agentPtrs.erase(childIter);
            }

            freeAgentPtrs.push_back(make_pair(ownerChangeEvent.ownerId, childAgentPtr));

            iterToAgentList.erase(ownerChangeEvent.agentId);

            agentThreadNumbers.erase(ownerChangeEvent.agentId);
        }

        ownerChangeEvents.clear();
    }

    typedef list<pair<AgentIdType, shared_ptr<Agent> > >::const_iterator IterType;

    for(IterType iter = freeAgentPtrs.begin(); iter != freeAgentPtrs.end(); iter++) {
        const AgentIdType& ownerAgentId = (*iter).first;
        const shared_ptr<Agent>& agentPtr = (*iter).second;
        const AgentIdType agentId = agentPtr->GetAgentId();

        agentPtr->parentAgentId = ownerAgentId;

        if (ownerAgentId == MASTER_ANY_AGENT_ID) {
            ThreadPartition& threadPartition = threadPartitions[agentPtr->originalThreadNumber];
            list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;
            map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

            iterToAgentList[agentId] = agentPtrs.insert(agentPtrs.end(), agentPtr);
            agentThreadNumbers[agentId] = agentPtr->originalThreadNumber;

        } else {
            if (agentThreadNumbers.find(ownerAgentId) == agentThreadNumbers.end()) {
                cerr << "Error: parent agent changed owner." << endl;
                exit(1);
            }

            const size_t threadNumber = agentThreadNumbers[ownerAgentId];

            ThreadPartition& threadPartition = threadPartitions[threadNumber];
            map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

            list<shared_ptr<Agent> >::iterator listIter = iterToAgentList[ownerAgentId];
            shared_ptr<Agent> ownerAgentPtr = (*listIter);
            list<shared_ptr<Agent> >& agentPtrs = ownerAgentPtr->childAgentPtrs;

            iterToAgentList[agentId] = agentPtrs.insert(agentPtrs.end(), agentPtr);

            agentThreadNumbers[agentId] = threadNumber;
        }
    }
}

void MultiAgentSimulator::DeleteInactiveAgents()
{
    for(size_t i = 0; i < threadPartitions.size(); i++) {
        ThreadPartition& threadPartition = threadPartitions[i];
        queue<AgentIdType>& deleteNodeIds = threadPartition.deleteNodeIds;
        list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;
        map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

        while (!deleteNodeIds.empty()) {
            const AgentIdType agentId = deleteNodeIds.front();

            typedef list<shared_ptr<Agent> >::iterator AgentIter;

            if (isDebugMode) {
                cout << "delete agent " << agentId << endl;
            }

            AgentIter agentIter = iterToAgentList[agentId];
            shared_ptr<Agent> agentPtr = (*agentIter);

            if (!agentPtr->HasParent()) {
                agentPtrs.erase(agentIter);
            } else {

                if (iterToAgentList.find(agentPtr->parentAgentId) != iterToAgentList.end()) {
                    AgentIter parentIter = iterToAgentList[agentPtr->parentAgentId];
                    shared_ptr<Agent> parentAgentPtr = (*parentIter);

                    parentAgentPtr->childAgentPtrs.erase(agentIter);
                }
            }

            assert(agentPtr->childAgentPtrs.empty());

            iterToAgentList.erase(agentId);

            deleteNodeIds.pop();

            (*this).DeleteNode(agentId);

            if (agentPtr->vehicleNodePtr !=nullptr) {
                shared_ptr<VehicleNode> vehicleNodePtr = agentPtr->vehicleNodePtr;

                reservedVehicleNodePtrs.at(vehicleNodePtr->GetVehicleType()).push(vehicleNodePtr);
            }
        }
    }
}

void MultiAgentSimulator::IncrementTimeStep()
{
    // master thread
    (*this).IncrementTimeStep(MASTER_THREAD_NUMBER);

    // increment at the end of time step for GUI step compatibility.
    currentSnapshotId = (currentSnapshotId + 1) % NUMBER_TIMESTEP_SNAPSHOTS;
}

void MultiAgentSimulator::IncrementTimeStep(const size_t threadNumber)
{
    typedef list<shared_ptr<Agent> >::const_iterator AgentIter;

    ThreadPartition& threadPartition = threadPartitions.at(threadNumber);

    const list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;

    queue<AgentIdType>& deleteNodeIds = threadPartition.deleteNodeIds;

    while (true) {

        timeIncrementThreadBarrier.wait();

        if (isSimulationDone) {
            break;
        }

        // before agent time incrementation synchronize topologies

        while (true) {
            shared_ptr<PreSynchronizer> preSynchronizerPtr;
            bool success;

            (*this).TakePreSynchronizer(preSynchronizerPtr, success);

            if (!success) {
                break;
            }

            preSynchronizerPtr->Synchronize();
            threadPartition.tookSynchronizerPtrs.push(preSynchronizerPtr);
        }

        timeIncrementThreadBarrier.wait();

        // main routin
        for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {

            Agent& agent = *(*iter);

            agent.IncrementTime(threadNumber);

            if (agent.IsDeletedAfterEndOfTimeStep()) {

                const AgentIdType& agentId = agent.GetAgentId();
                if (newlyAddedAgentIds.find(agentId) == newlyAddedAgentIds.end()) {
                    deleteNodeIds.push(agentId);
                    notEscapeAgent++;
                    assert(notEscapeAgent <= agentAddCount);
//                    cout << "end Agent:" << agent.GetAgentId() << "\tagentCount" << agentAddCount << endl;
                    //if(escapeAgentInfoTemp[agentAddCount - 1].getObjectAgentId() == agent.GetAgentId()){
                    //Outpu to CSV
                    if(notEscapeAgent >= agentAddCount){
//                        fstream writing_file, writing_file_congestion;
//                        writing_file.open("./result.csv", std::ios::out);   //current directory:[visuallab/scemtemp~~/sim] in visuallab
//                        writing_file_congestion.open("./result2_congestion.csv", std::ios::out);
//                        
//                        // 被災者情報リストCSVの作成
//                        for(int i=0; i < agentAddCount; i++){
//                            if(i == 0){
//                                writing_file << "AgentId,Congestion,TotalTravelTime,TotalTravelGain,TurnOffCount,shelterPrefix,";
//                                for(int j=0; j < escapeAgentInfoTemp[i].getShelterNumber(); j++)
//                                    writing_file << "shelterGainNo" << j << ",";
//                                writing_file << endl;
//                            }
//                            cout << escapeAgentInfoTemp[i].getObjectAgentId() << "I want to get agentTotalDistance::"
//                                    << escapeAgentInfoTemp[i].totalTravelGain << endl;
//                            writing_file << escapeAgentInfoTemp[i].getObjectAgentId() << ","    //AgentId
//                                        << escapeAgentInfoTemp[i].congestion << "," //Congestion
//                                        << escapeAgentInfoTemp[i].totalTravelTime << ","    //TotalTravelTime
////                                        << escapeAgentInfoTemp[i].getAgentGain(escapeAgentInfoTemp[i].getShelterPrefix()) << ","
//                                        << escapeAgentInfoTemp[i].totalTravelTime * -1.0 << "," //TotalTravelGain
//                                        << escapeAgentInfoTemp[i].getTurnOffCount() << ","  //TurnOff Count(たらい回し)
//                                        << escapeAgentInfoTemp[i].getShelterPrefix() << ",";    //ObjectiveShelterId
//                            for(int j=0; j < escapeAgentInfoTemp[i].getShelterNumber(); j++){   //each shelterGain
//                                writing_file << escapeAgentInfoTemp[i].getAgentGain(j) << ",";
//
//                            }
//                            writing_file << endl;
//                        }
//                        
//                        // 時系列混雑リストCSVの作成
//                        for(int i=0; i < 2000; i++){
//                            if(i == 0){
//                                writing_file_congestion << "Time(every10seconds), congestion" << endl;
//                            }
//                            writing_file_congestion << i*10 << "," << congestionTimeList[i] << endl;
//                        }
                        //exit(1);
                    }
                }
            }
        }

        timeIncrementThreadBarrier.wait();

        if (threadNumber == MASTER_THREAD_NUMBER) {
            (*this).ReturnPreSynchronizer();
            break;
        }
    }
}

void MultiAgentSimulator::TakePreSynchronizer(
    shared_ptr<PreSynchronizer>& preSynchronizerPtr,
    bool& success)
{
    success = false;

    if (preSynchronizerPtrs.empty()) {
        return;
    }

    boost::mutex::scoped_lock aLocker(preSynchronizerMutex);

    if (!preSynchronizerPtrs.empty()) {
        preSynchronizerPtr = preSynchronizerPtrs.front();
        preSynchronizerPtrs.pop();
        success = true;
    }
}

void MultiAgentSimulator::ReturnPreSynchronizer()
{
    for(size_t i = 0; i < threadPartitions.size(); i++) {
        queue<shared_ptr<PreSynchronizer> >& tookSynchronizerPtrs =
            threadPartitions[i].tookSynchronizerPtrs;

        while (!tookSynchronizerPtrs.empty()) {
            preSynchronizerPtrs.push(tookSynchronizerPtrs.front());
            tookSynchronizerPtrs.pop();
        }
    }
}

void MultiAgentSimulator::RerouteAllAgents()
{
    const TimeType currentTime = (*this).CurrentTime();

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        const ThreadPartition& threadPartition = threadPartitions[i];
        const list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;

        typedef list<shared_ptr<Agent> >::const_iterator AgentIter;

        for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {
            Agent& agent = *(*iter);

            agent.RecalculateRoute(currentTime);
        }
    }
}

static const GisObjectIdType MINIMUM_GIS_OBJECT_ID = 100000000;

void Agent::OutputTraceEvent()
{
    if (simEngineInterfacePtr != nullptr &&
        simEngineInterfacePtr->TraceIsOn(TraceMas)) {

        if (utility1Trace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "Utility1", utility1Trace.GetValueAndUnsetChangeFlag());
        }
        if (utility2Trace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "Utility2" , utility2Trace.GetValueAndUnsetChangeFlag());
        }
        if (travelDistanceTrace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "TravelDistance" , travelDistanceTrace.GetValueAndUnsetChangeFlag());
        }
        if (travelTimeTrace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "TravelTime" , travelTimeTrace.GetValueAndUnsetChangeFlag());
        }
        if (destinationChangeTrace.HasChanged() &&
            destPositionId != UNREACHABLE_POSITION_ID) {
            simulatorPtr->OutputTraceEvent(agentId, "DestinationChangeCount" , static_cast<double>(destinationChangeTrace.GetValueAndUnsetChangeFlag()));

            const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
            const string name = subsystem.GetGisObject(destPositionId).GetObjectName();

            simulatorPtr->OutputStringTraceEvent(agentId, "Destination" , name);
        }
        if (destinationChangeByCommunicationTrace.HasChanged() &&
            destPositionId != UNREACHABLE_POSITION_ID) {

            simulatorPtr->OutputTraceEvent(agentId, "DestinationChangeCountByCommunication" , static_cast<double>(destinationChangeByCommunicationTrace.GetValueAndUnsetChangeFlag()));
        }
    }
}


void MultiAgentSimulator::OutputTraceEvent(
    const NodeIdType& gisObjectOrAgentId,
    const string& eventName,
    const double value) const
{
    TraceSubsystem& traceSubsystem = theSimulationEnginePtr->GetTraceSubsystem();

    if (traceSubsystem.BinaryOutputIsOn()) {

        const MultiAgentTraceRecord record(value);

        traceSubsystem.OutputTraceInBinary(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            reinterpret_cast<const unsigned char* >(&record),
            sizeof(record),
            MASTER_THREAD_NUMBER);

    } else {

        ostringstream outStream;
        outStream << "V= " << value;

        traceSubsystem.OutputTrace(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            outStream.str(),
            MASTER_THREAD_NUMBER);
    }
}

void MultiAgentSimulator::OutputStringTraceEvent(
    const NodeIdType& gisObjectOrAgentId,
    const string& eventName,
    const string& value) const
{
    TraceSubsystem& traceSubsystem = theSimulationEnginePtr->GetTraceSubsystem();

    if (traceSubsystem.BinaryOutputIsOn()) {
        StateTraceRecord record;

        const size_t nameSize = std::min(value.size(), sizeof(record.stateId) - 1);

        for(size_t i = 0; i < nameSize; i++) {
            record.stateId[i] = value[i];
        }
        record.stateId[nameSize] = '\0';

        traceSubsystem.OutputTraceInBinary(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            reinterpret_cast<const unsigned char* >(&record),
            sizeof(record),
            MASTER_THREAD_NUMBER);

    } else {

        ostringstream outStream;
        outStream << "V= " << value;

        traceSubsystem.OutputTrace(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            outStream.str(),
            MASTER_THREAD_NUMBER);
    }
}

void MultiAgentSimulator::AttachCommunicationNode(
    const AgentIdType& agentId,
    const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    bool found = false;

    for(size_t i = 0; (!found && i < threadPartitions.size()); i++) {
        const ThreadPartition& threadPartition = threadPartitions[i];
        const map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

        typedef map<AgentIdType, list<shared_ptr<Agent> >::iterator>::const_iterator IterType;

        IterType iter = iterToAgentList.find(agentId);

        if (iter != iterToAgentList.end()) {
            (*(*iter).second)->AddCommunicationNode(communicationNodePtr);
            found = true;
        }
    }

    if (!found) {
        communicationNodePtrsWaitingAgentCreation[agentId].push_back(communicationNodePtr);
    }
}

void MultiAgentSimulator::DetachCommunicationNode(
    const AgentIdType& agentId,
    const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    bool found = false;

    for(size_t i = 0; (!found && i < threadPartitions.size()); i++) {
        const ThreadPartition& threadPartition = threadPartitions[i];
        const map<AgentIdType, list<shared_ptr<Agent> >::iterator>& iterToAgentList = threadPartition.iterToAgentList;

        typedef map<AgentIdType, list<shared_ptr<Agent> >::iterator>::const_iterator IterType;

        IterType iter = iterToAgentList.find(agentId);

        if (iter != iterToAgentList.end()) {
            (*(*iter).second)->DeleteCommunicationNode(communicationNodePtr);
            found = true;
        }
    }

    if (!found) {
        communicationNodePtrsWaitingAgentCreation[agentId].remove(communicationNodePtr);
    }
}

void MultiAgentSimulator::OutputAgentTraces()
{
    (*this).OutputTraceForAllNodePositions(ZERO_TIME);

    for(size_t i = 0; i < threadPartitions.size(); i++) {
        const ThreadPartition& threadPartition = threadPartitions[i];
        const list<shared_ptr<Agent> >& agentPtrs = threadPartition.agentPtrs;

        typedef list<shared_ptr<Agent> >::const_iterator AgentIter;

        for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {
            (*iter)->OutputTraceEvent();
        }
    }
}

void MultiAgentSimulator::CreateApplicationForNode(
    const AgentResource& resource,
    const NodeIdType& sourceNodeId,
    const InterfaceOrInstanceIdType& instanceId,
    const vector<string>& parameterLines,
    const set<NodeIdType>& targetNodeIds)
{
    ThreadPartition& threadPartition = threadPartitions[resource.ThreadNumber()];

    threadPartition.dynamicApplicationDatas.push_back(
        DynamicApplucationData(
            sourceNodeId,
            instanceId,
            parameterLines,
            targetNodeIds));
}

void MultiAgentSimulator::SyncApplicationCreation()
{
    for(size_t i = 0; i < threadPartitions.size(); i++) {
        ThreadPartition& threadPartition = threadPartitions[i];
        const vector<DynamicApplucationData>& dynamicApplicationDatas = threadPartition.dynamicApplicationDatas;

        for(size_t j = 0; j < dynamicApplicationDatas.size(); j++) {
            const DynamicApplucationData& dynamicApplicationData = dynamicApplicationDatas[j];

            (*this).GenerateDynamicApplication(dynamicApplicationData);
        }

        threadPartition.dynamicApplicationDatas.clear();
    }
}

void MultiAgentSimulator::GenerateDynamicApplication(const DynamicApplucationData& dynamicApplicationData)
{
    const NodeIdType& sourceNodeId = dynamicApplicationData.sourceNodeId;
    const InterfaceOrInstanceIdType& instanceId = dynamicApplicationData.instanceId;
    const vector<string>& parameterLines = dynamicApplicationData.parameterLines;
    const set<NodeIdType>& targetNodeIds = dynamicApplicationData.targetNodeIds;

    assert(!parameterLines.empty());

    for(size_t i = 0; i < parameterLines.size(); i++) {
        const string& parameterLine = parameterLines[i];

        bool foundAnError;

        theParameterDatabaseReaderPtr->AddNewDefinitionToDatabase(parameterLine, foundAnError);

        if (foundAnError) {
            cerr << "Error: Failed to add dynamic application line " << parameterLine << endl;
            exit(1);
        }
    }

    using std::dynamic_pointer_cast;

    if (targetNodeIds.find(ANY_NODEID) != targetNodeIds.end()) {

        typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

        for(IterType iter = nodes.begin(); iter != nodes.end(); iter++) {

            const shared_ptr<BasicNetworkNode> basicNetworkNodePtr =
                dynamic_pointer_cast<BasicNetworkNode>((*iter).second);

            if (basicNetworkNodePtr != nullptr) {
                basicNetworkNodePtr->CreateDynamicApplication(
                    *theParameterDatabaseReaderPtr,
                    theGlobalNetworkingObjectBag,
                    sourceNodeId,
                    instanceId);
            }
        }

    } else {

        typedef set<NodeIdType>::const_iterator IterType;

        for(IterType iter = targetNodeIds.begin(); iter != targetNodeIds.end(); iter++) {
            const NodeIdType& nodeId = (*iter);

            if (nodes.find(nodeId) == nodes.end()) {
                cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
                exit(1);
            }

            const shared_ptr<BasicNetworkNode> basicNetworkNodePtr =
                dynamic_pointer_cast<BasicNetworkNode>(nodes[nodeId]);

            if (basicNetworkNodePtr == nullptr) {
                cerr << "Error: Network Node Id: " << nodeId << " is not BasicNetworkNode." << endl;
                exit(1);
            }

            basicNetworkNodePtr->CreateDynamicApplication(
                *theParameterDatabaseReaderPtr,
                theGlobalNetworkingObjectBag,
                sourceNodeId,
                instanceId);
        }
    }
}

//---------------------------------------------------------------------------------

#pragma warning(disable:4355)

AgentProfileAndTaskTable::AgentProfileAndTaskTable(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const AgentResource& masterResource,
    const size_t numberThreads)
    :
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    emptyTaskTablePtr(new AgentTaskTable(this))
{
    const string agentProfileFilePath =
        initParameterDatabaseReader.ReadString(
            "multiagent-profile-file");

    const TimeType startTime =
        initParameterDatabaseReader.ReadTime(
            "multiagent-start-time");

    const double startTimeSec = static_cast<double>(startTime / SECOND);

    (*this).LoadProfile(
        *initGisSubsystemPtr, masterResource, agentProfileFilePath, startTimeSec);
}

#pragma warning(default:4355)

void AgentProfileAndTaskTable::CompleteInitialize(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const AgentResource& masterResource,
    const set<AgentIdType>& entireAgentIds)
{
    const string agentScheduleFilePath =
        initParameterDatabaseReader.ReadString(
            "multiagent-behavior-file");

    const TimeType startTime =
        initParameterDatabaseReader.ReadTime(
            "multiagent-start-time");

    const double startTimeSec = static_cast<double>(startTime / SECOND);

    map<string, set<string> > availableTaskTables;

    typedef set<AgentIdType>::const_iterator IterType;

    for(IterType iter = entireAgentIds.begin(); iter != entireAgentIds.end(); iter++) {

        const AgentIdType& agentId = (*iter);

        const string profileName = MakeLowerCaseString(
            initParameterDatabaseReader.ReadString(
                "multiagent-profile-type", agentId));

        const string taskTableName = MakeLowerCaseString(
            initParameterDatabaseReader.ReadString(
                "multiagent-behavior-type", agentId));

        if (AStringStartsWith(profileName, "taxi") ||
            profileName == "train" ||
            profileName == "bus" ||
            profileName == "vehicle" ||
            profileName == "privatecar") {
            // skip.
        } else {
            availableTaskTables[taskTableName].insert(profileName);
        }
    }

    (*this).LoadTaskTable(
        *initGisSubsystemPtr, masterResource, agentScheduleFilePath, startTimeSec, availableTaskTables);
}

shared_ptr<AgentProfile> AgentProfileAndTaskTable::GetProfile(const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        cerr << "Error: invalid profile name " << profileName << endl;
        exit(1);
    }

    return profilePtrs[profileName];
}

shared_ptr<AgentTaskTable> AgentProfileAndTaskTable::GetTaskTable(
        const string& taskTableName,
        const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        cerr << "Error: invalid profile name " << profileName << " for task " << taskTableName << endl;
        exit(1);
    }

    const AgentProfileType profileType = profilePtrs.GetId(profileName);

    typedef map<pair<string, AgentProfileType>, shared_ptr<AgentTaskTable> >::const_iterator IterType;

    IterType iter = taskTablePtrs.find(make_pair(taskTableName, profileType));

    if (iter == taskTablePtrs.end()) {
        cerr << "Specify available Behavior for Agent:" << taskTableName <<  " Profile:" << profileName << endl;
        exit(1);
    }

    return (*iter).second;
}

bool AgentProfileAndTaskTable::ContainsTask(
    const string& taskTableName,
    const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        return false;
    }

    const AgentProfileType profileType = profilePtrs.GetId(profileName);

    typedef map<pair<string, AgentProfileType>, shared_ptr<AgentTaskTable> >::const_iterator IterType;

    IterType iter = taskTablePtrs.find(make_pair(taskTableName, profileType));

    return (iter != taskTablePtrs.end());
}

vector<string> AgentProfileAndTaskTable::GetProfileTypeNames() const
{
    vector<string> profileTypeNames;

    for(size_t i = 0; i < profilePtrs.Size(); i++) {
        profileTypeNames.push_back(profilePtrs[AgentProfileType(i)]->GetProfileName());
    }

    return profileTypeNames;
}

void AgentProfileAndTaskTable::LoadProfile(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& masterResource,
    const string& profileFilePath,
    const double startTimeSec)
{
    ifstream inStream(profileFilePath.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open Profile file " << profileFilePath << endl;
        exit(1);
    }

    AgentProfileType profileType = INVALID_AGENT_TYPE;
    set<string> specifiedparameterNames;

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        if (AStringStartsWith(MakeLowerCaseString(aLine), "profiletype")) {
            const string profileName = SeparateString(aLine,  ":").second;

            if (profileName.empty()) {
                cerr << "Error can't read agent line " << aLine << endl;
                exit(1);
            }

            profileType = profilePtrs.GetId(MakeLowerCaseString(profileName));

            profilePtrs[profileType].reset(new AgentProfile(profileType, profileName));
            specifiedparameterNames.clear();
            continue;
        }

        ConvertStringToLowerCase(aLine);

        if (profileType == INVALID_AGENT_TYPE) {
            cerr << "Error:Define Profile in AgenrProfile file." << endl;
            exit(1);
        }

        deque<string> parameterNameAndValue;
        TokenizeToTrimmedLowerString(aLine, "=", parameterNameAndValue);

        if (parameterNameAndValue.size() != 2) {
            cerr << "Error:Paremter definition for " << profilePtrs.GetLabel(profileType) << " :" << aLine << endl;
            exit(1);
        }

        const string& parameterName = parameterNameAndValue[0];
        const string& value = parameterNameAndValue[1];

        if (specifiedparameterNames.find(parameterName) != specifiedparameterNames.end()) {
            cerr << "Error: Profile \"" << parameterName << "\" is duplicated in \"" <<  profilePtrs.GetLabel(profileType) << "\"" << endl;
            exit(1);
        }//if//

        specifiedparameterNames.insert(parameterName);

        if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_PRIVATE_CAR_OWNERSHIP]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.hasCar =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_BICYCLE_OWNERSHIP]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.hasBicycle =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_WALK_SPEED_AT_TRANSFER]) {

            const string& mobilityType = value;
            AgentProfile& profile = *profilePtrs[profileType];

            profile.mobilityClass = GetAgentMobilityClass(mobilityType);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY1_FUNCTION] || parameterName == "utilityfunction"/*obsolete*/) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.utilityFormula1 =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY2_FUNCTION]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.utilityFormula2 =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_ROUTE_PRIORITY]) {
            AgentProfile& profile = *profilePtrs[profileType];

            const AgentValueFormula aFormula(profile.parameters, startTimeSec, value, charactorIds);

            if (parameterName == "routepriority-pedestrian" ||
                parameterName == "routepriority-walk") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_PEDESTRIAN] = aFormula;

            } else if (parameterName == "routepriority-bicycle") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_BICYCLE] = aFormula;

            } else if (parameterName == "routepriority-privatecar") {

                profile.routeCostFormulas[ AGENT_BEHAVIOR_VEHICLE] = aFormula;

            } else if (parameterName == "routepriority-train") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_TRAIN] = aFormula;

            } else if (parameterName == "routepriority-bus") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_BUS] = aFormula;

            } else if (parameterName == "routepriority-taxi") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_TAXI] = aFormula;

            } else {

                for(size_t i = 0; i < profile.routeCostFormulas.size(); i++) {
                    if (profile.routeCostFormulas[i].IsNull()) {
                        profile.routeCostFormulas[i] = aFormula;
                    }
                }
            }

        } else {
            AgentProfile& profile = *profilePtrs[profileType];

            double defaultValue = 0;

            const AgentStatusIdType statusId = profile.parameters.GetId(parameterName);

            if (AGENT_RESERVED_STATUS_QUERY_TRIGGER_START <= statusId &&
                statusId <= AGENT_RESERVED_STATUS_QUERY_TRIGGER_END ) {
                defaultValue = DBL_MAX;
            }

            profile.parameters[parameterName] =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds, defaultValue);
        }
    }
}

static inline
bool IsEqualString(const string& aString)
{
    const size_t equalPos = aString.find("=");

    if (equalPos != string::npos) {

        const size_t conditionPos = aString.find_first_of("=<>!", equalPos + 1);

        if (conditionPos == equalPos + 1) {
            return false;
        }

        return true;
    }

    return false;
}

void AgentProfileAndTaskTable::LoadTaskTable(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& masterResource,
    const string& behaviorFilePath,
    const double startTimeSec,
    const map<string, set<string> >& availableTaskTables)
{
    ifstream inStream(behaviorFilePath.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open Behavior file " << behaviorFilePath << endl;
        exit(1);
    }

    //AgentTaskTableType taskTableType = INVALID_TASK_TABLE_TYPE;
    string taskTableName;
    vector<string> profileNames;
    vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > > defaultProfileTaskTablePtrs;

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }
        ConvertStringToLowerCase(aLine);

        if (AStringStartsWith(aLine, "locationgroup")) {

            const pair<string, string>& groupNameAndlocationNames =
                SeparateString(aLine.substr(string("locationgroup").length()),  "=");

            deque<string> locationNames;
            TokenizeToTrimmedLowerString(groupNameAndlocationNames.second, ",", locationNames);

            vector<GisPositionIdType>& positionIds =
                locationGroups[groupNameAndlocationNames.first];

            for(size_t i = 0; i < locationNames.size(); i++) {
                const GisPositionIdType positionId = theGisSubsystem.GetPosition(locationNames[i]);

                if (positionId.type == GIS_BUILDING ||
                    positionId.type == GIS_PARK ||
                    positionId.type == GIS_POI ||
                    positionId.type == GIS_AREA) {
                    positionIds.push_back(positionId);
                }
                else {
                    cerr << "Error: Group location can contain building, park, POI or area names only. name:" << theGisSubsystem.GetGisObject(positionId).GetObjectName() << endl;
                    exit(1);
                }
            }

        } else if (AStringStartsWith(aLine, "locationidgroup")) {

            const pair<string, string>& groupNameAndlocationNames =
                SeparateString(aLine.substr(string("locationidgroup").length()),  "=");

            deque<string> locationNames;
            TokenizeToTrimmedLowerString(groupNameAndlocationNames.second, ",", locationNames);

            vector<GisPositionIdType>& positionIds =
                locationGroups[groupNameAndlocationNames.first];

            for(size_t i = 0; i < locationNames.size(); i++) {
                bool success;
                int aValue;
                ConvertStringToInt(locationNames[i], aValue, success);

                if (!success) {
                    cerr << "Wrong format string for int value" << locationNames[i] << endl;
                    exit(1);
                }

                positionIds.push_back(
                    theGisSubsystem.GetPositionId(GisObjectIdType(aValue)));
            }

        } else if (AStringStartsWith(aLine, "behaviortype")) {
            const string behaviorName = SeparateString(aLine,  ":").second;

            if (behaviorName.empty()) {
                cerr << "Error can't read agent line " << aLine << endl;
                exit(1);
            }

            typedef map<string, set<string> >::const_iterator TaskTableIter;

            TaskTableIter taskTableIter = availableTaskTables.find(behaviorName);

            if (taskTableIter != availableTaskTables.end()) {
                taskTableName = behaviorName;

                typedef set<string>::const_iterator ProfileNameIter;

                const set<string>& availableProfileNames = (*taskTableIter).second;

                defaultProfileTaskTablePtrs.clear();

                for(ProfileNameIter nameIter = availableProfileNames.begin();
                    nameIter != availableProfileNames.end(); nameIter++) {

                    const string& profileName = (*nameIter);

                    if (!profilePtrs.Contains(profileName)) {
                        cerr << "Error: invalid profile name " << profileName << endl;
                        exit(1);
                    }

                    const AgentProfileType profileType = profilePtrs.GetId(profileName);

                    defaultProfileTaskTablePtrs.push_back(
                        make_pair(profileType,
                                  shared_ptr<AgentTaskTable>(new AgentTaskTable(this))));

                    taskTablePtrs[make_pair(behaviorName, profileType)] =
                        defaultProfileTaskTablePtrs.back().second;
                }

            } else {
                taskTableName.clear();
            }

        } else if (!taskTableName.empty()) {

            (*this).AddTask(
                theGisSubsystem,
                taskTableName,
                masterResource,
                aLine,
                startTimeSec,
                defaultProfileTaskTablePtrs,
                locationGroups);
        }
    }
}

static inline
void SeparateToConditionAndActionString(
    const string& taskLine,
    string& conditionString,
    string& actionString)
{
    size_t numberRemainingArcs = 0;
    size_t currentPos = 0;
    size_t conditionStartPos = 1;
    size_t conditionEndPos = 0;

    do {
        currentPos = taskLine.find_first_of("[]", currentPos);

        if (currentPos != string::npos) {
            if (taskLine[currentPos] == '[') {
                if (numberRemainingArcs == 0) {
                    conditionStartPos = currentPos + 1;
                }
                numberRemainingArcs++;
            } else {
                if (numberRemainingArcs == 0) {
                    cerr << "Error: lack of \"[\":" << taskLine << endl;
                    exit(1);
                }
                if (numberRemainingArcs == 1) {
                    conditionEndPos = currentPos - 1;
                }
                numberRemainingArcs--;
            }
            currentPos++;
        }

    } while (numberRemainingArcs != 0 && currentPos != string::npos);

    if (conditionEndPos >= conditionStartPos) {
        conditionString = taskLine.substr(conditionStartPos, conditionEndPos - conditionStartPos + 1);
        actionString = taskLine.substr(conditionEndPos + 2);
    } else {
        conditionString = "";
        actionString = taskLine;
    }
}

static inline
void ResolveApplicationParameterSpecification(
    const string& parameteLine,
    string& instanceName,
    string& modelName,
    vector<DynamicApplicationDefinitionParameter>& parameters)
{
    parameters.clear();

    string applicationSpecificaitonLine;

    SeparateToConditionAndActionString(parameteLine, instanceName, applicationSpecificaitonLine);

    deque<string> parameterTokens;
    TokenizeToTrimmedLowerStringWithArc(applicationSpecificaitonLine, ",", parameterTokens);

    for(size_t i = 0; i < parameterTokens.size(); i++) {
        const pair<string, string>& parameterAndValue =
            SeparateString(parameterTokens[i],  "=");

        parameters.push_back(
            DynamicApplicationDefinitionParameter(
                parameterAndValue.first,
                parameterAndValue.second));
    }

    if (!parameters.empty()) {
        const pair<string, string>& parameterAndValue =
            SeparateString(parameters.front().applicationParameterName,  "-");

        modelName = parameterAndValue.first;
    } else {
        modelName = "";
    }
}

static inline
void ParseActionLine(const string& actionLine, deque<string>& actionStrings)
{
    actionStrings.clear();

    size_t equalPos = 0;
    size_t delimPos = 0;

    // Quote is replace by white space according to parsing.
    string simplifiedActionLine  = actionLine;

    while (true) {
        const size_t actionStringStartPos = delimPos;

        equalPos = simplifiedActionLine.find_first_of('=', delimPos);

        if (equalPos == string::npos) {
            break;
        }

        bool valueIsOutOfQuote = true;
        int numberRemainingArcs = 0;

        delimPos = equalPos + 1;

        while (true) {

            delimPos = simplifiedActionLine.find_first_of(",\"()", delimPos);

            if (delimPos == string::npos) {
                break;
            }

            if (simplifiedActionLine[delimPos] == '(') {
                numberRemainingArcs++;
                delimPos++;
                continue;
            }

            if (simplifiedActionLine[delimPos] == ')') {
                numberRemainingArcs--;
                delimPos++;
                continue;
            }

            if (valueIsOutOfQuote &&
                numberRemainingArcs == 0 &&
                simplifiedActionLine[delimPos] == ',') {

                actionStrings.push_back(
                    simplifiedActionLine.substr(
                        actionStringStartPos,
                        delimPos - actionStringStartPos));

                delimPos++;
                break;
            }

            if (simplifiedActionLine[delimPos] == '"') {
                valueIsOutOfQuote = !valueIsOutOfQuote;
                simplifiedActionLine[delimPos] = ' '; //replace
            }

            delimPos++;
        }

        if (numberRemainingArcs != 0) {
            cerr << "Error: Mismatched number of function arcs in " << actionLine << endl;
            exit(1);
        }

        if (delimPos == string::npos) {

            if (!valueIsOutOfQuote) {
                cerr << "Error: invalid behavior line: [" << actionLine << "] Check number of \"" << endl;
                exit(1);
            }

            // found a specification(;equal) but no separator --> push last action
            actionStrings.push_back(
                simplifiedActionLine.substr(actionStringStartPos));

            break;
        }
    }

    for(size_t i = 0; i < actionStrings.size(); i++) {
        ConvertStringToLowerCase(actionStrings[i]);
    }
}

void AgentProfileAndTaskTable::AddTask(
    const GisSubsystem& theGisSubsystem,
    const string& taskTableName,
    const AgentResource& masterResource,
    const string& taskLine,
    const double startTimeSec,
    const vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > >& defaultProfileTaskTablePtrs,
    const map<string, vector<GisPositionIdType> >& locationGroups)
{
    string conditionLine;
    string actionLine;

    SeparateToConditionAndActionString(taskLine, conditionLine, actionLine);

    deque<string> conditionStrings;
    TokenizeToTrimmedLowerStringWithArc(conditionLine, ",", conditionStrings);

    deque<string> actionStrings;
    ParseActionLine(actionLine, actionStrings);

    vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > > availableProfileTaskTablePtrs;
    bool foundProfileTypeSpecification = false;

    for(size_t i = 0; (!foundProfileTypeSpecification && i < conditionStrings.size()); i++) {

        const string& conditionString = conditionStrings[i];

        if (IsEqualString(conditionString)) {
            const pair<string, string>& parameterAndValue =
                SeparateString(conditionString,  "=");

            if (parameterAndValue.first == "profiletype") {
                foundProfileTypeSpecification = true;

                deque<string> profileNames;
                TokenizeToTrimmedLowerStringWithArc(parameterAndValue.second, ";", profileNames);

                for(size_t j = 0; j < profileNames.size(); j++) {

                    const string& profileName = profileNames[j];

                    if (!profilePtrs.Contains(profileName)) {
                        cerr << "Error: invalid profile name " << profileName << " at " << conditionLine << endl;
                        exit(1);
                    }

                    const AgentProfileType profileType = profilePtrs.GetId(profileName);

                    availableProfileTaskTablePtrs.push_back(
                        make_pair(profileType,
                                  taskTablePtrs[make_pair(taskTableName, profileType)]));
                }
            }
        }
    }
    if (!foundProfileTypeSpecification) {
        availableProfileTaskTablePtrs = defaultProfileTaskTablePtrs;
    }

    for(size_t i = 0; i < availableProfileTaskTablePtrs.size(); i++) {
        const pair<AgentProfileType, shared_ptr<AgentTaskTable> >& profileTaskTablePtr =
                   availableProfileTaskTablePtrs[i];

        const AgentProfileType& profileType = profileTaskTablePtr.first;
        const AgentProfile& profile = *(profilePtrs[profileType]);

        AgentTaskTable& taskTable = *(profileTaskTablePtr.second);

        shared_ptr<AgentTask> taskPtr(new AgentTask(this));
        AgentTask& task = *taskPtr;

        for(size_t j = 0; j < conditionStrings.size(); j++) {
            const string& conditionString = conditionStrings[j];

            if (j == 0 && conditionString.find_first_of("=<>!") == string::npos) {

                task.startTime = AgentValueFormula(
                    profile.parameters, startTimeSec, conditionString, charactorIds);

            } else {
                const size_t conditionPos = conditionString.find_first_of("=<>!");

                if (!(conditionPos > 0 && conditionPos != string::npos) ){
                    cerr << "Error can't read condition line " << taskLine << endl;
                    exit(1);
                }

                const string parameterName = TrimmedString(conditionString.substr(0, conditionPos - 1));

                if (parameterName == "profiletype") {
                    continue;
                }

                const string value = conditionString.substr(conditionPos);

                if (!profile.parameters.Contains(parameterName)) {
                    cerr << "Error: Profile Type " << profilePtrs.GetLabel(profileType)
                         << " doesn't have status parameter " << parameterName << endl;
                    exit(1);
                }

                const AgentStatusIdType statusId =
                    profile.parameters.GetId(parameterName);

                const AgentConditionChecker::ConditionParameterType parameterType =
                    AgentConditionChecker::ConvertToParameterType(statusId);

                task.conditionCheckers.push_back(
                    AgentConditionChecker(
                        profile.parameters,
                        parameterType,
                        startTimeSec,
                        value,
                        charactorIds));
            }
        }

        vector<pair<string, string> > parameterAndValues;

        bool isJustStatusChange = true;
        bool hasWaitingAction = false;
        bool isInterrputionBehavior = false;

        for(size_t j = 0; j < actionStrings.size(); j++) {
            const string& actionString = actionStrings[j];

            parameterAndValues.push_back(SeparateString(actionString,  "="));

            const string& parameterName = parameterAndValues.back().first;

            if (parameterName == "initiallocationid" ||
                parameterName == "initiallocation" ||
                parameterName == "movetodestinationid" ||
                parameterName == "movetodestination" ||
                parameterName == "waituntil" ||
                parameterName == "wait") {

                isJustStatusChange = false;

            } else if (parameterName == "interruptcurrentaction") {

                const string& value = parameterAndValues.back().second;

                if (value == "terminatenow" || value == "resumeafterinterruption") {
                    isInterrputionBehavior = true;
                }

            } else if (parameterName == "wait") {
                hasWaitingAction = true;
            }
        }

        bool isBeforeSpecification = true;
        bool isBeforeWaiting = true;

        bool specifiedDestinationChoiceType = false;

        for(size_t j = 0; j < parameterAndValues.size(); j++) {
            const pair<string, string>& parameterAndValue = parameterAndValues[j];
            const string& parameterName = parameterAndValue.first;
            const string& value = parameterAndValue.second;

            if (parameterName == "initiallocationid" ||
                parameterName == "initiallocation") {

                if (!taskTable.initialLocation.locationName.empty()) {
                    cerr << "Error: duplicated location definition for " << taskLine << endl;
                    exit(1);
                }
                if (value == "initiallocation") {
                    cerr << "Error: \"InitialLocation\" is a reserved destination string." << endl;
                    exit(1);
                }

                if (parameterName == "initiallocationid") {
                    taskTable.initialLocation.isId = true;
                }
                taskTable.initialLocation.locationName = value;

            } else if (parameterName == "neardestinationid" ||
                       parameterName == "neardestination") {

                cerr << "Error: Use \"Destination=" << value << "、DestinationChoiceType=Nearest\" specification instead of \"" << parameterName << "=" << value << "\"" << endl;
                    exit(1);

            } else if (parameterName == "movetodestinationid" ||
                       parameterName == "movetodestination") {

                if (!task.destination.locationName.empty()) {
                    cerr << "Error: duplicated destination definition for " << taskLine << endl;
                    exit(1);
                }

                task.destination.locationName = value;

                isBeforeSpecification = false;

            } else if (parameterName == "destinationchoicetype") {
                cout << "destinationChoiceType::" << value << endl;

                if (value == "nearest") {
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_NEAREST;
                } else if (value == "random") {
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_RANDOM;
                    //umeki add this part
                } else if (value == "knapsack"){
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_KNAPSACK;
                } else if (value == "knapsacksim"){
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_KNAPSACK_SIMULATION;
                    //Umeki add above
                } else {
                    cerr << "Error: invalid destination choice type " << value << endl;
                    exit(1);
                }
                specifiedDestinationChoiceType = true;

            } else if (parameterName == "departuretime") {

                task.departureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "earliestdeparturetime") {

                task.earlyDepartureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "latestdeparturetime") {

                task.lateDepartureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "arrivaltime") {

                task.arrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "earliestarrivaltime") {

                task.earlyArrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "latestarrivaltime") {

                task.lateArrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "waituntil") {

                task.endTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

                isBeforeSpecification = false;

            } else if (parameterName == "wait") {

                task.waitTime = AgentValueFormula(
                    profile.parameters, 0, value, charactorIds);

                isBeforeWaiting = false;

            } else if (parameterName == "preferedmobilitymeans") {

                if (value == "pedestrian" || value == "walk") {
                    task.preferedBehavior = AGENT_BEHAVIOR_PEDESTRIAN;
                } else if (value == "bicycle") {
                    task.preferedBehavior = AGENT_BEHAVIOR_BICYCLE;
                } else if (value == "vehicle" || value == "car" || value == "privatecar") {
                    task.preferedBehavior = AGENT_BEHAVIOR_VEHICLE;
                } else if (value == "taxi") {
                    task.preferedBehavior = AGENT_BEHAVIOR_TAXI;
                } else if (value == "bus") {
                    task.preferedBehavior = AGENT_BEHAVIOR_BUS;
                } else if (value == "train") {
                    task.preferedBehavior = AGENT_BEHAVIOR_TRAIN;
                } else {
                    cerr << "Error: invalid PreferedMobility<eans " << value << endl;
                    exit(1);
                }

            } else if (parameterName == "mobilitymeans") {

                if (value == "pedestrian" || value == "walk") {
                    task.behavior = AGENT_BEHAVIOR_PEDESTRIAN;
                } else if (value == "bicycle") {
                    task.behavior = AGENT_BEHAVIOR_BICYCLE;
                } else if (value == "vehicle" || value == "car" || value == "privatecar") {
                    task.behavior = AGENT_BEHAVIOR_VEHICLE;
                } else if (value == "taxi") {
                    task.behavior = AGENT_BEHAVIOR_TAXI;
                } else if (value == "bus") {
                    task.behavior = AGENT_BEHAVIOR_BUS;
                } else if (value == "train") {
                    task.behavior = AGENT_BEHAVIOR_TRAIN;
                } else {
                    cerr << "Error: invalid Mobility<eans " << value << endl;
                    exit(1);
                }

            } else if (parameterName == "reservation") {

                if (!IsFunctionString(value)) {
                    cerr << "Error: invalid reservation  " << value << endl;
                    exit(1);
                }

                const size_t arcPos = value.find_first_of('(');
                const string functionString = value.substr(0, arcPos);

                cerr << "Error: invalid reservation function " << functionString << endl;
                exit(1);

            } else if (parameterName == "intersectionstogothrough") {

                deque<string> passIntersectionNames;
                TokenizeToTrimmedLowerString(value, ":", passIntersectionNames);

                for(size_t k = 0; k < passIntersectionNames.size(); k++) {
                    task.passIntersectionLocationInfos.push_back(
                        LocationInfo(
                            false/*isId*/,
                            AGENT_LOCATION_CHOICE_RANDOM,
                            passIntersectionNames[k]));
                }

            } else if (parameterName == "interruptcurrentaction") {

                if (value == "terminatenow") {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_REPLACE;
                } else if (value == "resumeafterinterruption") {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_LATER;
                } else {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_NONE;
                }

            } else if (parameterName == "generateapplication") {

                string applicationModelName;
                string instanceName;
                vector<DynamicApplicationDefinitionParameter> applicationParameters;

                ResolveApplicationParameterSpecification(
                    value,
                    instanceName,
                    applicationModelName,
                    applicationParameters);

                const AgentStatusChangeType changeType = GetStatusChangeType(
                    isJustStatusChange,
                    hasWaitingAction,
                    isBeforeSpecification,
                    isInterrputionBehavior,
                    isBeforeWaiting);

                const DynamicApplicationIdType dynamicApplicationId(
                    applicationModelName,
                    instanceName);

                task.additionalStatusChanges[changeType].applicationSpecifications[dynamicApplicationId].parameters = applicationParameters;

            } else {
                //assert(profile.parameters.Contains(parameterName));

                if (!profile.parameters.Contains(parameterName)) {
                    cerr << "Warning: Couldn't find an agent profile parameter: " << parameterName << endl
                         << "   A profile change behavior [" << actionStrings[j] << "] is skippped." << endl;
                    continue;
                }

                const AgentStatusChangeType changeType = GetStatusChangeType(
                    isJustStatusChange,
                    hasWaitingAction,
                    isBeforeSpecification,
                    isInterrputionBehavior,
                    isBeforeWaiting);

                const AgentStatusIdType statusId =
                    profile.parameters.GetId(parameterName);

                task.additionalStatusChanges[changeType].statusChanges.push_back(
                    make_pair(statusId, AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds)));
            }
        }

        if (isInterrputionBehavior) {
            if (isJustStatusChange) {
                cerr << "Error: Specify destination or wait for InterruptCurrentAction " << actionLine << endl;
                exit(1);
            }
            taskTable.interruptTaskPtrs.push_back(taskPtr);

        } else {

            if (isJustStatusChange) {
                taskTable.statusChangePtrs.push_back(taskPtr);
            } else {
                taskTable.taskPtrs.push_back(taskPtr);
            }
        }

        if (taskTable.initialLocation.locationName.empty()) {
            cerr << "Error: Set initial location for Behavior Type " << taskTableName << " Profile Type " << profilePtrs.GetLabel(profileType) << endl;
            exit(1);
        }
    }
}

AgentValueFormula AgentProfileAndTaskTable::MakeValueFormula(
    const AgentProfileType& profileType,
    const string& formula)
{
    const AgentProfile& profile = *profilePtrs[profileType];

    const double defaultValue = 0;
    const double startTimeSec = 0;

    return AgentValueFormula(profile.parameters, startTimeSec, formula, charactorIds, defaultValue);
}


}//namespace