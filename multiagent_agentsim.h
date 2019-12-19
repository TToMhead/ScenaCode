// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_AGENTSIM_H
#define MULTIAGENT_AGENTSIM_H

#include "multiagent_common.h"
#include "multiagent_support.h"
#include "scensim_netsim.h"
#include <deque>

namespace MultiAgent {

using ScenSim::NetworkSimulator;
using ScenSim::SimulationEvent;
using ScenSim::ShapeSensingModel;
using ScenSim::FanSensingShape;
using ScenSim::SensingSharedInfoType;
using ScenSim::SensingModel;

class AgentValueFormula;
struct AgentStatusChangeEvent;
class AgentProfileAndTaskTable;

const double MinStepDistanceToCountInStatsMeters = 1e-10;
const int MAX_N = 100000; // n(人数)の最大値
const int MAX_W = 50000; // 各避難所の許容量の最大値
const int MAX_S = 1000; // Shelterの最大値
const int MAX_SIM = 10000;  //指定するシミュレーション時間



bool AStringStartsWith(const string& aLine, const string& aString);

struct LocationInfo {
    bool isId;
    AgentLocationChoiceType locationChoiceType;
    string locationName;

    LocationInfo()
        :
        isId(false),
        locationChoiceType(AGENT_LOCATION_CHOICE_RANDOM)
    {}

    LocationInfo(
        const bool initIsId,
        const AgentLocationChoiceType& initLocationChoiceType,
        const string& initLocationName)
        :
        isId(initIsId),
        locationChoiceType(initLocationChoiceType),
        locationName(initLocationName)
    {}
};


template <typename IdType, typename ValueType>
class LabelMap {
public:
    IdType GetId(const string& name) const {
        typedef map<string, size_t>::const_iterator IterType;
        IterType iter = ids.find(name);//MakeLowerCaseString(name));

        if (iter == ids.end()) {
            cerr << "Error: Couldn't find an agent profile parameter: " << name << endl;
            exit(1);
        }

        return IdType((*iter).second);
    }

    IdType GetId(const string& name) {

        //const string lowerName = MakeLowerCaseString(name);

        if ((*this).Contains(name)) {
            return (IdType)(ids[name]);
        } else {
            const IdType id = (IdType)(ids.size());

            ids[name] = id;
            labels.push_back(name);
            values.push_back(ValueType());

            return id;
        }
    }

    bool Contains(const string& name) const {
        return (ids.find(MakeLowerCaseString(name)) != ids.end());
    }

    const ValueType& operator[](const IdType& id) const { return values.at(size_t(id)); }

    ValueType& operator[](const string& name) {
        typedef map<string, size_t>::iterator IterType;

        //const string lowerName = MakeLowerCaseString(name);

        IterType iter = ids.find(name);

        if (iter == ids.end()) {
            ids[name] = values.size();
            labels.push_back(name);
            values.push_back(ValueType());
            return values.back();
        }

        return values[(*iter).second];
    }

    const ValueType& operator[](const string& name) const {
        typedef map<string, size_t>::const_iterator IterType;

        IterType iter = ids.find(name);

        if (iter == ids.end()) {
            cerr << "Error: Couldn't find an agent profile parameter: " << name << endl;
            exit(1);
        }

        return values.at((*iter).second);
    }

    ValueType& operator[](const IdType& id) { return values.at(id); }

    const string& GetLabel(const IdType& id) const { return labels[id]; }

    size_t Size() const { return values.size(); }
    size_t size() const { return values.size(); }

    bool empty() const { return values.empty(); }

private:
    map<string, size_t> ids;
    vector<string> labels;
    vector<ValueType> values;
};


class Agent {
public:
    Agent(
        MultiAgentSimulator* initSimulatorPtr,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const AgentIdType& initAgentId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr,
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr);
    ~Agent();

    static shared_ptr<Agent> CreateMasterAgent(
        MultiAgentSimulator* initSimulatorPtr,
        const AgentIdType& initAgentId,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr);

    static shared_ptr<Agent> CreateTaxiDriverAgent(
        MultiAgentSimulator* initSimulatorPtr,
        const AgentIdType& initAgentId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr,
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
        const shared_ptr<VehicleNode>& initVehicleNodePtr,
        const shared_ptr<Taxi>& initTaxiPtr);

    static shared_ptr<Agent> CreateBusDriverAgent(
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
        const shared_ptr<Bus>& initBusPtr);

    static shared_ptr<Agent> CreateTrainDriverAgent(
        MultiAgentSimulator* initSimulatorPtr,
        const AgentIdType& initAgentId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr,
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
        const shared_ptr<VehicleNode>& initVehicleNodePtr,
        const shared_ptr<Train>& initTrainPtr);

    TimeType CalculateWakeupTime();

    void IncrementTime(const size_t threadNumber);

    //ObjectMobilityPosition GetPosition(const TimeType) {
    //    (status[nextStep].pos - status[lastStep].pos()) * completionRation + status[lastStep].pos();
    //}

    bool IsDeletedAfterEndOfTimeStep() const { return isDeletedAtTheEndOfTimeStep; }

    AgentIdType GetAgentId() const { return agentId; }

    bool HasCar() const { return (vehiclePtr != nullptr); }
    bool HasBicycle() const { return hasBicycle; }

    shared_ptr<Vehicle> GetVehicle() const { return vehiclePtr; }

    void OutputTrace(const string& aString) const;

    shared_ptr<SimulationEngineInterface> GetSimEngineInterfacePtr() const { return simEngineInterfacePtr; }

    shared_ptr<ObjectMobilityModel> GetMobilityModelPtr() const { return mobilityModelPtr; }

    shared_ptr<AgentProfile> GetProfilePtr() const { return profilePtr; }


    void AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr);
    void DeleteCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr);

    void RecalculateRoute(
        const TimeType& recalculateStartTime,
        const AgentBehaviorType& initRecalculateRouteWithBehavior = AGENT_BEHAVIOR_ANY);

    void OutputTraceEvent();

    void GetStartAndDestVertexPairs(
        const VertexIdType& prioritizedStartVertexId,
        const VertexIdType& prioritizedDestVertexId,
        vector<VertexIdType>& startVertexIds,
        vector<VertexIdType>& destVertexIds);

    void GetNearRouteSearchCandidateVertexIds(
        const GisPositionIdType& positionId,
        const Vertex& position,
        const VertexIdType& prioritizedDestVertexId,
        vector<VertexIdType>& destVertexIds);

    void ReadyToDestruct();

private:
    Agent(
        MultiAgentSimulator* initSimulatorPtr,
        const AgentIdType& initAgentId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr,
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
        const shared_ptr<VehicleNode>& initVehicleNodePtr,
        const shared_ptr<Vehicle>& initVehiclePtr = shared_ptr<Vehicle>());

    Agent(
        MultiAgentSimulator* initSimulatorPtr,
        const AgentIdType& initAgentId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<AgentProfile>& initProfilePtr,
        const shared_ptr<AgentTaskTable>& initTaskTablePtr);

    void ExecuteNewTaskIfNecessary();

    void ApplyStatusChangesAndInstantiateApplications();
    void ApplyAdditionalStatusChanges(const AgentAdditionalStatusChange& additionalStatusChange);
    void ApplyStatusChanges(const vector<pair<AgentStatusIdType, AgentValueFormula> >& statusChanges);
    void InstantiateApplications(
        const map<DynamicApplicationIdType, DynamicApplicationDefinition>& applicationSpecifications);


    void InitializeStatusWith(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
        const bool calculateCarAndBicycleProbability,
        bool& hasCar);

    void AssignCurrentTask();
    void AssignInterruptTask();
    void GoToNextBehaviorIfPossible();

    void DecideCurrentBehavior();
    void IncrementCurrentBehaviorTime();

    void UpdateHealthFactor();
    void UpdateUtility();

    //bool NeedToRecalculateRoute() const;
    void RecalculateBehaviorIfNecessary(bool& assignedNewBehavior);

    void DecideRoute(const AgentBehaviorType& specifiedBehavior = AGENT_BEHAVIOR_ANY);
    void DecideRoute(
        const VertexIdType& startVertexId,
        const VertexIdType endVertexId,
        const AgentBehaviorType& specifiedBehavior,
        bool& foundRoute);

    bool ShouldChangeRouteInCurrentBehavior(const AgentBehaviorType& specifiedBehavior) const;

    double CalculateRouteWeight(const AgentRouteList& aRoute);

    void SetDestination(
        const GisPositionIdType& initDestPositionId,
        const bool initCanChangeToOtherDestinationCandidate,
        const bool byCommunication);
    void SetCurrentDestinationToUnreachablePosition();
    void AddUnreachablePositions(
        const list<GisPositionIdType>& unreachablePositionIds,
        const bool byCommunication);

    void ChangeToSpecificDestination(
        const GisPositionIdType& initDestPositionId,
        const VertexIdType& initDestVertexId,
        const bool byCommunication);

    void SetVertexId(const VertexIdType& vertexId);

    bool CurrentTaskHasFinished() const;
    bool HasCurrentTask() const;
    bool HasParent() const { return parentAgentId != MASTER_ANY_AGENT_ID; }
    bool WaitingAtDestinationEntrace() const;
    bool WaitingAtEntrance() const;
    bool OtherDestinationSeemsToBeBetter();

    const Vertex& GetCurrentPosition() const;
    const Vertex& GetNextPosition() const;

    const AgentTask& CurrentTask() const;
    const GisPositionIdType& GetHomePositinId() const;

    friend class AgentResource;
    friend class MultiAgentSimulator;

    MultiAgentSimulator* simulatorPtr;
    shared_ptr<MultiAgentGis> theAgentGisPtr;
    shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr;
    shared_ptr<AgentRouteSearchSubsystem> theRouteSearchSubsystemPtr;

    static const string modelName;
    const AgentIdType agentId;
    const shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    const shared_ptr<AgentProfile> profilePtr;
    shared_ptr<AgentTaskTable> taskTablePtr;

    size_t currentThreadNumber;
    size_t originalThreadNumber;

    bool isDeletedAtTheEndOfTimeStep;

    AgentStatus status;
    shared_ptr<Vehicle> vehiclePtr;
    bool hasBicycle;

    TimeType currentTaskStartTime;
    TimeType currentTaskEndTime;
    bool isTaskInitialized;
    deque<VertexIdType> shouldPassVertexIds;

    priority_queue_stable<AgentStatusChangeEvent> timeLineStatusChangeEvents;
    size_t currentTaskNumber;

    bool currentIsInterruptTask;
    size_t currentInterruptTaskNumber;

    TimeToSearchRoute timeToSearchRoute;
    //vector<pair<GisPositionIdType, VertexIdType> > destinations;
    VertexIdType destVertexId;
    GisPositionIdType destPositionId;
    GisPositionIdType extraDestPoiId;//POI on destination park/building

    bool canChangeToOtherDestinationCandidate;
    set<GisPositionIdType> unreachableDestinationIds;

    RouteNumberType currentRouteNumber;
    AgentRouteList currentRouteList;
    //list<shared_ptr<AgentRoute> >::iterator currentRouteIter;

    GisPositionIdType homePositionId;

    shared_ptr<AgentBehavior> currentBehaviorPtr;
    VertexIdType lastVertexId;
    GisPositionIdType currentPositionId;
    GisPositionIdType extraCurrentPoiId;

    TimeType entranceWaitStartTime;

    double directionRadians;

    TimeType currentBehaviorStartTime;
    AgentHealthOrUtilityFactor healthOrUtilityFactor;

    TimeType lastRouteCalculatedTime;
    TimeType lastPathQueryTriggerTime;

    bool recalculateRoute;
    AgentBehaviorType recalculateRouteWithBehavior;
    set<AgentBehaviorType> notAvailableBehavorTypesForNextRouteCalculation;

    TimeType lastDelay;
    uint32_t utility1CalculationCount;
    uint32_t utility2CalculationCount;

    AgentRouteCost currentBehaviorCost;
    HighQualityRandomNumberGenerator aRandomNumberGenerator;// for general case
    HighQualityRandomNumberGenerator aRandomNumberGeneratorForDestinationChoice;

    static const int SEED_HASHING_FOR_DESTINATION_CHOICE = 543636;

    map<size_t, shared_ptr<BusTicket> > busTicketPtrs;

    AgentIdType parentAgentId;
    list<shared_ptr<Agent> > childAgentPtrs;

    shared_ptr<VehicleNode> vehicleNodePtr;

    // human interface for seeing, talking,...
    shared_ptr<ShapeSensingModel> humanInterfacePtr;

    // network interface
    set<shared_ptr<AgentCommunicationNode> > communicationNodePtrs;
    shared_ptr<ObjectMobilityModel> mobilityModelPtr;

    shared_ptr<RealStatistic> utility1StatPtr;
    shared_ptr<RealStatistic> utility2StatPtr;

    shared_ptr<RealStatistic> travelDistanceStatPtr;
    shared_ptr<RealStatistic> travelTimeStatPtr;
    shared_ptr<CounterStatistic> numberNoRouteStatPtr;
    shared_ptr<CounterStatistic> numberRouteCalculateTimeStatPtr;

    TraceValue<double> utility1Trace;
    TraceValue<double> utility2Trace;

    TraceValue<double> congestionTrace;

    TraceValue<double> travelDistanceTrace;
    TraceValue<double> travelTimeTrace;

    TraceValue<uint32_t> destinationChangeTrace;
    TraceValue<uint32_t> destinationChangeByCommunicationTrace;
};//Agent//



class AgentResource {
public:
    AgentResource() : agentPtr(nullptr) {}
    AgentResource(Agent* initAgentPtr) : agentPtr(initAgentPtr) {}
    AgentResource(const shared_ptr<Agent>& initAgentPtr) : agentPtr(initAgentPtr.get()) {}

    bool IsAvailable() const { return (agentPtr != nullptr); }
    void CheckAgentAvailability() const {
        if (!(*this).IsAvailable()) {
            cerr << "Agent resource is empty. AgentResource APIs is initialized after an agent initialization time step." << endl;
            exit(1);
        }
    }

    size_t ThreadNumber() const { (*this).CheckAgentAvailability(); return agentPtr->currentThreadNumber; }

    TimeType CurrentTime() const;
    TimeType CurrentBehaviorSpentTime() const;

    AgentIdType AgentId() const { (*this).CheckAgentAvailability(); return agentPtr->agentId; }
    AgentProfileType ProfileType() const;
    string GetProfileName() const;

    AgentUserType UserType() const;
    AgentMobilityClassType MobilityClass() const;
    AgentTicketType TicketType() const;

    const GisPositionIdType& DestPositionId() const;
    const GisPositionIdType& ExtraDestPoiId() const;
    const GisPositionIdType& HomePositionId() const;
    RouteNumberType CurrentRouteNumber() const;
    const AgentRoute& CurrentRoute() const;
    StopIdType GetNextRouteStopId() const;
    const set<GisPositionIdType>& UnreachableDestinationIds() const;

    const AgentTaskPurposeType& CurrentPurpose() const;

    const VertexIdType& LastVertexId() const { (*this).CheckAgentAvailability();  return agentPtr->lastVertexId; }
    const GisPositionIdType& PositionId() const { (*this).CheckAgentAvailability();  return agentPtr->currentPositionId; }
    const VertexIdType& DestVertexId() const { (*this).CheckAgentAvailability();  return agentPtr->destVertexId; }

    const AgentStatus& Status() const { (*this).CheckAgentAvailability(); return agentPtr->status; }
    const Vertex& Position() const;

    ObjectMobilityPosition MobilityPosition() const;
    ObjectMobilityPosition MobilityPositionForTime(const TimeType& time) const;
    double Value(const AgentStatusIdType& id) const { return (*this).Status().values[id]; }

    AgentBehaviorType GetBehaviorType() const;

    int Age() const { return static_cast<int>((*this).Status().values[AGENT_RESERVED_STATUS_AGE]); }
    double WalkSpeedMetersPerSec() const { return (*this).Status().values[AGENT_RESERVED_STATUS_WALK_SPEED]; }
    double BicycleSpeedMetersPerSec() const { return (*this).Status().values[AGENT_RESERVED_STATUS_BICYCLE_SPEED]; }
    double MaxVehicleSpeedMetersPerSec() const { return (*this).Status().values[AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED]; }

    double LastDelayQueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_LAST_DELAY_QUERY_TRIGGER]; }
    double NextDelayQueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_NEXT_DELAY_QUERY_TRIGGER]; }
    double TripDelayQueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_TRIP_DELAY_QUERY_TRIGGER]; }
    double VehicleDelayQueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_VEHICLE_DELAY_QUERY_TRIGGER]; }
    double CongestionQueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_CONGESTION_QUERY_TRIGGER]; }

    double Utility1QueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_UTILITY1_QUERY_TRIGGER]; }
    double Utility2QueryTrigger() const { return (*this).Status().values[AGENT_RESERVED_STATUS_UTILITY2_QUERY_TRIGGER]; }

    TimeType MinPathQueryInterval() const { return static_cast<TimeType>((*this).Status().values[AGENT_RESERVED_STATUS_MIN_PATH_QUERY_INTERVAL])*SECOND; }
    double PathQueryProbability() const { return (*this).Status().values[AGENT_RESERVED_STATUS_PATH_QUERY_PROBABILITY]; }
    double MissedVehiclePathQueryProbability() const { return (*this).Status().values[AGENT_RESERVED_STATUS_MISSED_VEHICLE_PATH_QUERY_PROBABILITY]; }

    double GasolinCostPerMeters() const { return (*this).Status().values[AGENT_RESERVED_STATUS_GASOLIN_COST_PER_METERS]; }
    double Utility1() const { return (*this).Status().values[AGENT_RESERVED_STATUS_UTILITY1]; }
    double Utility2() const { return (*this).Status().values[AGENT_RESERVED_STATUS_UTILITY2]; }

    TimeType RidingTime() const { return static_cast<TimeType>((*this).Status().values[AGENT_RESERVED_STATUS_RIDING_TIME])*SECOND; }
    double SeeingPedestrianProbability() const { return (*this).Status().values[AGENT_RESERVED_STATUS_SEEING_PEDESTRIAN_PROBABILITY]; }
    double TimeHeadway() const { return (*this).Status().values[AGENT_RESERVED_STATUS_TIME_HEADWAY]; }
    double MinVehicleGap() const { return (*this).Status().values[AGENT_RESERVED_STATUS_MIN_VEHICLE_GAP]; }
    double MaxAcceleration() const { return (*this).Status().values[AGENT_RESERVED_STATUS_MAX_ACCELERATION]; }
    double MaxDeceleration() const { return (*this).Status().values[AGENT_RESERVED_STATUS_MAX_DECELERATION]; }
    double MaxBrakingDecceleration() const { return  (*this).Status().values[AGENT_RESERVED_STATUS_MAX_BRAKING_DECCELERATION]; }
    double AccelerationExponent() const { return  (*this).Status().values[AGENT_RESERVED_STATUS_ACCELERATION_EXPONENT]; }
    double SaveAcceleration() const { return  (*this).Status().values[AGENT_RESERVED_STATUS_SAVE_DECELERATION]; }
    double MaxTurnSpeedMetersPerSec() const { return  (*this).Status().values[AGENT_RESERVED_STATUS_MAX_TURN_SPEED]; }

    double LaneChangeAccelerationThreshold() const { return (*this).Status().values[AGENT_RESERVED_LANE_CHANGE_ACCELERATION_THRESHOLD]; }
    double VelocityRatioGapDistance() const { return (*this).Status().values[AGENT_RESERVED_VELOCITY_RATIO_GAP_DISTANCE]; }
    double OtherVehicleEntranceTime() const { return (*this).Status().values[AGENT_RESERVED_OTHER_VEHICLE_ENATRANCE_TIME]; }
    double PassiveYieldTime() const { return (*this).Status().values[AGENT_RESERVED_PASSIVE_YIELD_TIME]; }
    double ActiveYieldTime() const { return (*this).Status().values[AGENT_RESERVED_ACTIVE_YIELD_TIME]; }
    double YieldWaitingTime() const { return (*this).Status().values[AGENT_RESERVED_YIELD_WAITING_TIME]; }

    double TotalTravelDistance() const { return (*this).Status().values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_DISTANCE]; }
    double TotalTravelTime() const { return (*this).Status().values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_TIME]; }
    double RoadWidthWeight() const { return (*this).Status().values[AGENT_RESERVED_STATUS_ROAD_WIDTH_WEIGHT]; }
    bool IsDisasterMode() const { return ((*this).Status().values[AGENT_RESERVED_STATUS_DISASTER] > 0); }
    TimeType RouteRecalculationTime() const { return static_cast<TimeType>((*this).Status().values[AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME])*SECOND; }
    
    //Tanaka added ProfileのHandicappedプロパティ
    bool Handicapped() const { return ((*this).Status().values[AGENT_RESERVED_STATUS_HANDICAPPED] > 0); }
    double AcceptableWalkDistanceToCar() const { return (*this).Status().values[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_CAR]; }
    double AcceptableWalkDistanceToStop() const { return (*this).Status().values[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_STOP]; }

    double MinVehicleRouteDistance() const { return (*this).Status().values[AGENT_RESERVED_MIN_VEHICLE_ROUTE_DISTANCE]; }
    int NumberMaxRouteCandidates() const { return static_cast<int>((*this).Status().values[AGENT_RESERVED_NUMBER_MAX_ROUTE_CANDIDATES]); }
    int NumberPeople() const { return static_cast<int>((*this).Status().values[AGENT_RESERVED_NUMBER_PEOPLE]); }
    TimeType EntranceWaitTime() const { return static_cast<TimeType>((*this).Status().values[AGENT_RESERVED_ENTRANCE_WAIT_TIME])*SECOND; }
    TimeType TaxiCallWaitTime() const { return static_cast<TimeType>((*this).Status().values[AGENT_RESERVED_TAXICALL_WAIT_TIME])*SECOND; }

    bool WaitingAtEntrance() const;
    bool ExceededWaitEntranceTime() const;

    VehicleConstant MakeVehicleConstant(const double vehicleLengthMeters = DEFAULT_VEHICLE_LENGTH_METERS) const {
        return VehicleConstant(
            vehicleLengthMeters*0.5,
            (*this).LaneChangeAccelerationThreshold(),
            (*this).MaxAcceleration(),
            (*this).MaxDeceleration(),
            (*this).MaxBrakingDecceleration(),
            (*this).AccelerationExponent(),
            (*this).SaveAcceleration(),
            (*this).MaxTurnSpeedMetersPerSec(),
            (*this).MaxVehicleSpeedMetersPerSec(),
            (*this).MinVehicleGap(),
            (*this).TimeHeadway(),
            (*this).VelocityRatioGapDistance(),
            (*this).OtherVehicleEntranceTime(),
            (*this).PassiveYieldTime(),
            (*this).ActiveYieldTime(),
            (*this).YieldWaitingTime(),
            (*this).GasolinCostPerMeters());
    }

    TimeType LastDelay() const;
    TimeType NextDelay() const;
    TimeType TripDelay() const;
    TimeType ArrivalDelay() const;

    double Congestion() const { return agentPtr->healthOrUtilityFactor.values[AGENT_HEALTH_FACTOR_CONGESTION]; }

    bool HasCar() const { return agentPtr->HasCar(); }
    bool HasBicycle() const { return agentPtr->hasBicycle; }
    bool CanUseBicycle() const { return (*this).HasBicycle(); }
    bool IsPathQueryTriggerAvailable(
        const double congestion = 0,
        const TimeType& vehicleDelay = ZERO_TIME) const;

    const Vehicle& GetVehicle() const { return *agentPtr->vehiclePtr; }

    void SetLastPathQueryTriggerTime();

    bool ExceededRouteRecalculationTime() const;

// special function for taxi
    void AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr);

//private functions: MUST not call from others

    AgentStatus& Status() { return agentPtr->status; }

    void SetPosition(const Vertex& position);
    void SetVertexId(const VertexIdType& vertexId);
    void SetPositionId(const GisPositionIdType& positionId);
    void SetExtraPoiId(const GisPositionIdType& positionId);
    void SetPositionId(const GisObjectType& objectType, const VariantIdType& variantId);
    void SetDirectionRadians(const double directionRadians);

    void SetCongestion(const double value);

    void RecalculateRoute(const TimeType& recalculateStartTime);
    void RecalculateRoute();
    void RecalculateRouteWithBehavior(const AgentBehaviorType& behavior);

    void RecalculateRouteWithNotAvailableBehaviorSpecification(
        const set<AgentBehaviorType>& notAvailableBehaviorTypes);
    void ArrivedAtDeadEndNotification();
    void UnreachableDestinationNotification();
    void ArrivedAtDestinationNotification();
    void EnteredToDestinationNotification();
    void ArrivedAtGisPositionNotification();

    void SetDestination(
        const Vertex& position,
        const bool byCommunication);

    void SetDestination(
        const GisPositionIdType& positionId,
        const Vertex& position,
        const bool byCommunication);

    void AddUnreachablePositions(const list<GisPositionIdType>& unreachablePositionIds, const bool byCommunication);

    void ReceivePhysicalData(SensingSharedInfoType& broadcastData);

    void SetOwnerAgent(
        const AgentIdType& ownerAgentId = MASTER_ANY_AGENT_ID);

    void RemoveOwnerAgent();

    void WaitEntrance();
    void AllowedEntrance();

    HighQualityRandomNumberGenerator& GetRandomNumberGenerator() const;
    HighQualityRandomNumberGenerator& GetRandomNumberGeneratorForDestinationChoice() const;

    bool operator==(const AgentResource& right) const { return (*this).AgentId() == right.AgentId(); }
    bool operator<(const AgentResource& right) const { return (*this).AgentId() < right.AgentId(); }


    // for debuging
    void OutputTrace(const string& aString) const { return agentPtr->OutputTrace(aString); }
    const Vertex& DebugNextPosition() const;

private:
    Agent* agentPtr;
};

class AgentValueFormula {
public:
    AgentValueFormula(const double initDefaultvalue = 0) : defaultValue(initDefaultvalue) {}

    AgentValueFormula(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& statusValues,
        const double simStartTimeSec,
        const string& aString,
        map<string, AgentCharactorIdType>& charactorIds,
        const double initDefaultvalue = 0);

    AgentValueFormula(
        const string& aString,
        const double initDefaultvalue = 0);

    bool IsNull() const { return formulaUnits.empty(); }

    double CalculateDouble(
        const AgentResource& resource,
        const bool calculateMaxValue = false) const;

    int CalculateInt(
        const AgentResource& resource,
        const bool calculateMaxValue = false) const {
        return static_cast<int>((*this).CalculateDouble(resource, calculateMaxValue));
    }

    TimeType CalculateTime(
        const AgentResource& resource,
        const bool calculateMaxValue = false) const;

    double CalculateUtility(
        const AgentResource& resource,
        const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
        const AgentRouteCost& cost) const;

    class Formula {
    public:
        virtual ~Formula() {}
        virtual double operator()(
            const vector<double>& values,
            const AgentResource& resource) const = 0;
    };

    string GetInputFormulaString() const { return inputFormulaString; }

private:
    typedef size_t FormulaUnitIdType;

    static const FormulaUnitIdType NO_FORMULA_UNIT_ID = BAD_SIZE_T;

    typedef uint8_t FormulaOperationType;
    enum {
        FORMULA_OPERATION_NONE = 0,

        FORMULA_OPERATION_PLUS = 1,
        FORMULA_OPERATION_MINUS = 2,
        FORMULA_OPERATION_DIV = 3,
        FORMULA_OPERATION_MULTI = 4,
        FORMULA_OPERATION_MOD = 5,
        FORMULA_OPERATION_E_PLUS = 6,
        FORMULA_OPERATION_E_MINUS = 7,

        FORMULA_OPERATION_LOG10 = 8,
        FORMULA_OPERATION_LOGN = 9,
        FORMULA_OPERATION_POW = 10,
        FORMULA_OPERATION_MIN = 11,
        FORMULA_OPERATION_MAX = 12,
        FORMULA_OPERATION_SQRT = 13,
        FORMULA_OPERATION_SIN = 14,
        FORMULA_OPERATION_COS = 15,
        FORMULA_OPERATION_TAN = 16,
        FORMULA_OPERATION_ABS = 17,
        FORMULA_OPERATION_CEIL = 18,
        FORMULA_OPERATION_FLOOR = 19,
        FORMULA_OPERATION_PI = 20,
        FORMULA_OPERATION_EXP = 21,

        FORMULA_OPERATION_DISTRIBUTION_START = 22,

        FORMULA_OPERATION_UNI = 22,
        FORMULA_OPERATION_UNID = 23,
        FORMULA_OPERATION_NORMAL = 24,
        FORMULA_OPERATION_EXP_DISTRIBUTION = 25,
        FORMULA_OPERATION_POISSON = 26,
        FORMULA_OPERATION_ERLANG = 27,

        NUMBER_FORMULA_TYPES,
    };

    struct FormulaUnit {
        FormulaUnitIdType unitId;
        FormulaOperationType operation;
        vector<pair<FormulaUnitIdType, double> > values;

        FormulaUnit(const FormulaUnitIdType& initUnitId)
            :
            unitId(initUnitId)
        {}

        FormulaUnit(
            const FormulaUnitIdType& initUnitId,
            const FormulaOperationType& initOperation)
            :
            unitId(initUnitId),
            operation(initOperation)
        {}

        bool CompletedAllCalculation() const;
    };

    string inputFormulaString;
    vector<FormulaUnit> formulaUnits;
    double defaultValue;

    // shared_array
    static const shared_ptr<Formula> formulaPtrs[];

    FormulaUnit& GetFormulaUnit(const FormulaUnitIdType& unitId);
    const FormulaUnit& GetFormulaUnit(const FormulaUnitIdType& unitId) const;

    FormulaOperationType GetFormulaOperation(
        const string& functionString,
        const size_t numberArguments) const;

    void PrecalculateFormulaUnit(const FormulaUnitIdType& unitId);

    double Calculate(
        const FormulaUnitIdType& unitId,
        const AgentResource& resource,
        const bool calculateMaxValue = false) const;

    double CalculateUtility(
        const FormulaUnitIdType& unitId,
        const AgentResource& resource,
        const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
        const AgentRouteCost& cost) const;

    double CalculateMax(
        const FormulaUnitIdType& unitId,
        const AgentResource& resource) const;

    FormulaUnitIdType AddFormulaUnitRecursively(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
        const double simStartTimeSec,
        const string& aString,
        map<string, AgentCharactorIdType>& charactorIds);

    FormulaUnitIdType AddFunctionUnitRecursively(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
        const double simStartTimeSec,
        const string& aString,
        map<string, AgentCharactorIdType>& charactorIds);

    FormulaUnitIdType AddVariableUnit(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& statusValues,
        const string& aString);

    static void ResolveOperation(
        const string& aString,
        string& leftValue,
        string& rightValue,
        FormulaOperationType& operation);


    static const int ROUTE_COST_START = NUMBER_FORMULA_TYPES;
    static const int HEALTH_OR_UTILITY_FACTOR_START = ROUTE_COST_START + NUMBER_AGENT_ROUTE_COSTS;
    static const int AGENT_STATUS_START = HEALTH_OR_UTILITY_FACTOR_START + NUMBER_HEALTH_OR_UTILITY_FACTORS;

    static bool IsRouteCost(const FormulaOperationType& operation) {
        return (ROUTE_COST_START <= operation && operation < HEALTH_OR_UTILITY_FACTOR_START);
    }
    static bool IsHealthOrUtilityCost(const FormulaOperationType& operation) {
        return (HEALTH_OR_UTILITY_FACTOR_START <= operation && operation < AGENT_STATUS_START);
    }
    static bool IsAgentStatus(const FormulaOperationType& operation) {
        return (operation >= AGENT_STATUS_START);
    }
    static bool MayBeConstValue(const FormulaOperationType& operation) {
        return (operation <= NUMBER_FORMULA_TYPES);
    }

    static FormulaOperationType ConvertTofFormulaOperationType(const AgentStatusIdType& statusId) {
        return FormulaOperationType(AGENT_STATUS_START + statusId);
    }

    static AgentRouteCostType ConvertToRouteCostId(const FormulaOperationType& operation) {
        return AgentRouteCostType(operation - ROUTE_COST_START);
    }
    static AgentHealthOrUtilityFactorType ConvertToHealthOrUtilityFactorId(const FormulaOperationType& operation) {
        return AgentHealthOrUtilityFactorType(operation - HEALTH_OR_UTILITY_FACTOR_START);
    }
    static AgentStatusIdType ConvertTofAgentStatusId(const FormulaOperationType& operation) {
        return AgentStatusIdType(operation - AGENT_STATUS_START);
    }
};

class AgentConditionChecker {
public:
    typedef  uint8_t ConditionParameterType;
    enum {
        CONDITION_PARAMETER_TIME,
        //CONDITION_PARAMETER_LOCATION,

        NUMBER_RESERVED_CONDITION_PARAMETERS,
    };

    AgentConditionChecker()
        :
        Checker(&AgentConditionChecker::Check0)
    {}

    AgentConditionChecker(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& initParameters,
        const ConditionParameterType& initParameterType,
        const double sinitSimStartTimeSec,
        const string& initString,
        map<string, AgentCharactorIdType>& initCharactorIds);

    bool Check(const AgentResource& resource) const;

    static ConditionParameterType ConvertToParameterType(const AgentStatusIdType& statusId) {
        return ConditionParameterType(statusId + NUMBER_RESERVED_CONDITION_PARAMETERS);
    }

    static AgentStatusIdType ConvertToStatusId(const ConditionParameterType& parameterType) {
        return AgentStatusIdType(parameterType - NUMBER_RESERVED_CONDITION_PARAMETERS);
    }

private:
    ConditionParameterType parameterType;
    AgentValueFormula formula;

    bool (AgentConditionChecker::*Checker)(const double& v1, const double& v2) const;

     bool Check0(const double& v1, const double& v2) const { return true; }
     bool Check1(const double& v1, const double& v2) const { return (v1 == v2); }
     bool Check2(const double& v1, const double& v2) const { return (v1 != v2); }
     bool Check3(const double& v1, const double& v2) const { return (v1 < v2); }
     bool Check4(const double& v1, const double& v2) const { return (v1 <= v2); }
     bool Check5(const double& v1, const double& v2) const { return (v1 > v2); }
     bool Check6(const double& v1, const double& v2) const { return (v1 >= v2); }
};

class  AgentTask {
public:
    AgentTask(AgentProfileAndTaskTable* initProfileAndTaskTablePtr)
        :
        profileAndTaskTablePtr(initProfileAndTaskTablePtr),
        purpose(AGENT_TASK_PURPOSE_OTHER),
        preferedBehavior(AGENT_BEHAVIOR_ANY),
        behavior(AGENT_BEHAVIOR_ANY),
        interruptionType(AGENT_BEHAVIOR_INTERRUPTION_NONE)
    {}

        
    TimeType GetStartTime(const AgentResource& resource) const;
    TimeType GetEndTime(const AgentResource& resource) const;
    TimeType GetWaitTime(const AgentResource& resource) const;
    bool SpecifiedArrivalTime() const { return !arrivalTime.IsNull(); }

    bool SatisfyCondition(const AgentResource& resource) const;

    void GetDestinationId(
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const bool ignoreLastPositionFromCandidate,
        GisPositionIdType& positionId,
        bool& isMultipleDestinations,
        AgentResource& resource) const;

    bool HasPassVertexSpecification() const { return !passIntersectionLocationInfos.empty(); }

    void GetPassVertexIds(
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        deque<VertexIdType>& passVertexIds,
        AgentResource& resource) const;

    const AgentAdditionalStatusChange& GetAdditionaStatusChange(const AgentStatusChangeType& changeType) const {
        typedef map<AgentStatusChangeType, AgentAdditionalStatusChange>::const_iterator IterType;

        IterType iter = additionalStatusChanges.find(changeType);

        assert(iter != additionalStatusChanges.end());

        return (*iter).second;
    }

    const bool HasStatusChange(const AgentStatusChangeType& changeType) const {
        return (additionalStatusChanges.find(changeType) != additionalStatusChanges.end());
    }

    const AgentTaskPurposeType& GetPurpose() const { return purpose; }
    const AgentBehaviorType& GetPreferedBehavior() const {
        if (behavior != AGENT_BEHAVIOR_ANY) {
            return behavior;
        }
        return preferedBehavior;
    }
    const AgentBehaviorInterruptionType& GetInterruptionType() const { return interruptionType; }

    const AgentBehaviorType& GetBehavior() const { return behavior; }

    void GetTimeLine(
        const AgentResource& resource,
        const TimeType& earlyStartTime,
        TimeToSearchRoute& timeToSearchRoute) const;

private:
    friend class AgentProfileAndTaskTable;
    AgentProfileAndTaskTable* profileAndTaskTablePtr;

    AgentValueFormula startTime;
    vector<AgentConditionChecker> conditionCheckers;

    LocationInfo destination;
    deque<LocationInfo> passIntersectionLocationInfos;

    AgentValueFormula endTime;
    AgentValueFormula waitTime;
    AgentValueFormula arrivalTime;
    AgentValueFormula earlyArrivalTime;
    AgentValueFormula lateArrivalTime;

    AgentValueFormula departureTime;
    AgentValueFormula earlyDepartureTime;
    AgentValueFormula lateDepartureTime;

    AgentTaskPurposeType purpose;
    AgentBehaviorType preferedBehavior;
    AgentBehaviorType behavior;
    AgentBehaviorInterruptionType interruptionType;

    map<AgentStatusChangeType, AgentAdditionalStatusChange> additionalStatusChanges;
};

struct AgentStatusChangeEvent {
    TimeType time;

    AgentStatusChangeType statusChangeType;
    size_t statusChangeNumber;

    bool operator<(const AgentStatusChangeEvent& right) const {
        return time > right.time;
    }

    AgentStatusChangeEvent()
        :
        time(ZERO_TIME),
        statusChangeType(AGENT_STATUS_CHANGE_BASIC_TASK_START),
        statusChangeNumber(0)
    {}

    AgentStatusChangeEvent(
        const TimeType& initTime,
        const size_t statusChangeNumber,
        const AgentStatusChangeType& initStatusChangeType)
        :
        time(initTime),
        statusChangeType(initStatusChangeType),
        statusChangeNumber(statusChangeNumber)
    {}
};

class AgentTaskTable {
public:
    AgentTaskTable(AgentProfileAndTaskTable* initProfileAndTaskTablePtr)
        :
        profileAndTaskTablePtr(initProfileAndTaskTablePtr)
    {}

    size_t GetNumberOfTasks() const { return taskPtrs.size(); }
    size_t GetNumberOfStatusChanges() const { return statusChangePtrs.size(); }
    size_t GetNumberOfInterruptTasks() const { return interruptTaskPtrs.size(); }

    void GetStatusChanges(
        const AgentResource& resource,
        priority_queue_stable<AgentStatusChangeEvent>& timeLineStatusChangeEvents) const;

    const AgentTask& GetTask(const size_t taskNumber) const { return *taskPtrs.at(taskNumber); }
    const AgentTask& GetStatusChange(const size_t statusChangeNumber) const { return *statusChangePtrs.at(statusChangeNumber); }
    const AgentTask& GetInterruptTask(const size_t interruptTaskNumber) const { return *interruptTaskPtrs.at(interruptTaskNumber); }

    void GetInitialLocationId(
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        GisPositionIdType& positionId,
        AgentResource& resource) const;

    void GetLocationId(
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const LocationInfo& locationInfo,
        GisPositionIdType& positionId,
        bool& isMultiplePositions,
        AgentResource& resource) const;

    bool IsEmpty() const { return taskPtrs.empty(); }

private:
    friend class AgentProfileAndTaskTable;
    AgentProfileAndTaskTable* profileAndTaskTablePtr;

    LocationInfo initialLocation;

    LabelMap<AgentStatusIdType, AgentValueFormula> parameters;
    vector<AgentValueFormula> routeCostFormulas;

    vector<shared_ptr<AgentTask> > taskPtrs;
    vector<shared_ptr<AgentTask> > statusChangePtrs;
    vector<shared_ptr<AgentTask> > interruptTaskPtrs;
};




class AgentProfile {
public:
    AgentProfile(
        const AgentProfileType& initProfileType,
        const string& initProfileName = "");

    const string& GetProfileName() const { return profileName; }

    const AgentProfileType& GetProfileType() const { return profileType; }
    const AgentUserType& GetUserType() const { return userType; }
    const AgentMobilityClassType& GetMobilityClass() const { return mobilityClass; }
    const AgentTicketType& GetTicketType() const { return ticketType; }

    double GetHasCarRatio(const AgentResource& resource) const { return hasCar.CalculateDouble(resource); }
    double GetHasBicycleRatio(const AgentResource& resource) const { return hasBicycle.CalculateDouble(resource); }

    const LabelMap<AgentStatusIdType, AgentValueFormula>& GetParameters() const { return parameters; }

    double CalculateRouteUtility(
        const AgentResource& resource,
        const AgentBehaviorType& behavior,
        const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
        const AgentRouteCost& cost) const {
        return routeCostFormulas.at(behavior).CalculateUtility(resource, healthOrUtilityFactor, cost);
    }

    double CalculateUtility1(
        const AgentResource& resource,
        const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
        const AgentRouteCost& cost) const {
        return utilityFormula1.CalculateUtility(resource, healthOrUtilityFactor, cost);
    }
    double CalculateUtility2(
        const AgentResource& resource,
        const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
        const AgentRouteCost& cost) const {
        return utilityFormula2.CalculateUtility(resource, healthOrUtilityFactor, cost);
    }

    const AgentValueFormula& GetPrimaryRouteCostFormula() const {
        assert(!routeCostFormulas.empty());
        return routeCostFormulas.front();
    }

    const AgentValueFormula& GetUtilityFormula1() const { return utilityFormula1; }
    const AgentValueFormula& GetUtilityFormula2() const { return utilityFormula2; }

private:
    friend class AgentProfileAndTaskTable;

    string profileName;

    AgentProfileType profileType;
    AgentUserType userType;
    AgentMobilityClassType mobilityClass;
    AgentTicketType ticketType;

    AgentValueFormula hasCar;
    AgentValueFormula hasBicycle;

    AgentValueFormula utilityFormula1;
    AgentValueFormula utilityFormula2;

    LabelMap<AgentStatusIdType, AgentValueFormula> parameters;
    vector<AgentValueFormula> routeCostFormulas;
};



// Umeki add-----------------
struct EscapeAgentInfo {
  int agentId;
  double congestion;
  double totalTravelTime;
  double totalTravelGain;
  double totalTravelDistance;
  int shelterPrefix;
  bool notObeyFlag;
  int arrivedFlagCount;

  EscapeAgentInfo(
    const int inputAgentId)
    :
    agentId(inputAgentId),
    congestion(0),
    totalTravelGain(0),
    totalTravelTime(0),
    arrivedFlagCount(0),
    notObeyFlag(false)
  {}

  EscapeAgentInfo()
    :
    agentId(0),
    congestion(0),
    totalTravelGain(0),
    totalTravelTime(0),
    arrivedFlagCount(0),
    notObeyFlag(false)
  {}

  void setNotObeyFlag(bool flag){ notObeyFlag = flag; }
  bool getNotObeyFlag(){ return notObeyFlag; }
  void setAgentId(int i){ agentId = i; }
  int getAgentId(){ return agentId; }
  void setAgentGain(float Gain){ agentGain.push_back(Gain); }
  void clearAgentGain(){ agentGain.clear(); agentGain.shrink_to_fit(); }
  float getAgentGain(int i){ return agentGain[i]; }
  int checkAgentGainNumber(){ return agentGain.size(); }
  void ovrideAgentGain(int i, double inputGain){
      if(agentGain.size() < i  || agentGain.size() == 0){
          //RandomBuildingを使いたい場合はsim_default.exeを使用すること
          cout << "OutOfIndex Number of Agent::" << i << "of" << agentGain.size() << endl;
          cout << "Prease refer to multiagent_agentsim.h." << endl;
          cout << "It is probably due to set RandomBuilding the MoveToDestination.";
          exit(1);
      }
      agentGain[i] = inputGain;
  }
  int getShelterNumber(){ return agentGain.size(); }

  void setObjectAgentId(int inputObjectAgentId){ ObjectAgentId = inputObjectAgentId; }
  int getObjectAgentId(){ return ObjectAgentId; }

  void setShelterId(GisPositionIdType inputShelterId){ purShelterId = inputShelterId; }
  GisPositionIdType getShelterId(){ return purShelterId; }

  void setShelterPrefix(int input){ shelterPrefix = input; }
  int getShelterPrefix(){ return shelterPrefix; }
  
  void setPurposeShelterGain(double input){ purShelterGain = input; }
  double getPurposeShelterGain(){ return purShelterGain; }

  void setAgentResource(AgentResource inputResource){ agentResource = inputResource; }
  AgentResource getAgentResource(){ return agentResource; }

  void addTurnOffCount(){ TurnOffCount++; }
  int getTurnOffCount(){ return TurnOffCount; }
  
  void setArrivedFlag(bool flag){ arrivedFlag = flag; }
  bool getArrivedFlag(){ return arrivedFlag; }

private:
  int ObjectAgentId;
  int TurnOffCount = 0;
  bool arrivedFlag = false;   //目的地に到着しているかどうかを判定
  vector<double> agentGain; //添え字がShelterInfo.shelterIdに対応
  GisPositionIdType purShelterId;  //決定された避難所のObjectIdを管理(ShelterInfo.shelterIdと同じもの)
  double purShelterGain;  //決定された避難所への利得
  AgentResource agentResource;
};

struct ShelterInfo{

public:
  int shelterId;  //locationCandidateIdsの添え字を代入予定
  int currentPersonNumber;  //許容量の取得
  vector<double> agentGain; //避難所への利得のリスト
  vector<int> agentId;  //利得リストの中の添え字保存配列
  //vector<AgentResource> agentResources; //Tanaka added
  //vector<string> agentProfile;
  vector<int> agentProfile;

  ShelterInfo()
    :
    currentPersonNumber(0),
    peopleSuffix(0)
  {}

  void setAgentId(int i){ agentId.push_back(i); }
  int getAgentId(int i){ return agentId[i]; }
  void setAgentGain(double inputGain){ agentGain.push_back(inputGain); }
  double getAgentGain(int i){ return agentGain[i]; }
  void initializeTemp(int i){ tempId.assign(i,0); tempGain.assign(i,0); tempProfile.assign(i,0); }
  //void setAgentResource(AgentResource inputResource){ agentResources.push_back(inputResource); } //Tanaka added
  //void setAgentProfile(string inputProfile){ agentProfile.push_back(inputProfile); }
  void setAgentProfile(int i){ agentProfile.push_back(i); }
  //AgentResource getAgentResourcees(int i){ return agentResources[i]; } //Tanaka added
  //string getAgentProfile(int i){ return agentProfile[i]; }
  int getAgentProfile(int i){ return agentProfile[i]; }

  void setAllowableValue(int inputAllowableValue){ AllowableValue = inputAllowableValue; }
  int getAllowableValue(){ return AllowableValue; }
  void setShelterId(GisPositionIdType inputObjectShelterId){ ObjectShelterId = inputObjectShelterId; }
  GisPositionIdType getShelterId(){ return ObjectShelterId; }
  void addPerson(){ currentPersonNumber++; }
  int getCurrentPerson(){ return currentPersonNumber; }

  void addSuffix(){ peopleSuffix++; }
  int getSuffix(){ return peopleSuffix; }

  bool checkShelter(){
    if (AllowableValue == currentPersonNumber){ //収容限界に達した処理
      return false;
    }
    else if (AllowableValue < currentPersonNumber){ //限界越えてる処理．エラーのため強制終了
      cout << "Error: Exceeded allowable amount" << endl;
      exit(1);
    }
    else{ //収容可能の処理
      return true;
    }
  }

//"Profileが"
//void ProfileSort(int left, int right)
//  {
    
 // }


    /* 所持する利得リストを降順にソーティングする */
  void MergeSort(int left, int right)
  {
     int mid, i, j, k;

     if (left >= right)              /* 配列の要素がひとつなら */
         return;                     /* 何もしないで戻る */
                                     /* ここでは分割しているだけ */
     mid = (left + right) / 2;       /* 中央の値より */
     MergeSort(left, mid);        /* 左を再帰呼び出し */
     MergeSort(mid + 1, right);   /* 右を再帰呼び出し */

       /* x[left] から x[mid] を作業領域にコピー */
     for (i = left; i <= mid; i++){
         //temp[i] = x[i];
         tempGain.at(i) = agentGain[i];
         tempId.at(i) =  agentId[i];
         tempProfile.at(i) = agentProfile[i];
    }

       /* x[mid + 1] から x[right] は逆順にコピー */
     for (i = mid + 1, j = right; i <= right; i++, j--){
         //temp[i] = x[j];
         tempGain.at(i) = agentGain[j];
         tempId.at(i) =  agentId[j];
         tempProfile.at(i) = agentProfile[j];
     }

     i = left;         /* i とj は作業領域のデーターを */
     j = right;        /* k は配列の要素を指している */

     for (k = left; k <= right; k++){    /* 小さい方から配列に戻す */
         if (tempGain[i] >= tempGain[j]){        /* ここでソートされる */
             agentGain.at(k) = tempGain[i];
             agentId.at(k) =  tempId[i];
             agentProfile.at(k) = tempProfile[i];
             i++;
         }
         else{
             agentGain.at(k) = tempGain[j];
             agentId.at(k) =  tempId[j];
             agentProfile.at(k) = tempProfile[j];
             j--;
         }
     }
  }

private:
  int AllowableValue;  //収容限界数
  GisPositionIdType ObjectShelterId;
  vector<int> tempId;
  vector<double> tempGain;
  vector<int> tempProfile;
  int peopleSuffix;
  AgentResource agentResource;
};


//Knapsackを管理する際に必要な変数を管理するために用意したが，このままの扱いなら構造体名ミスった感が否めない
struct KnapsackInfo{
public:
  int numberAgent;
  int numberShelter;

  KnapsackInfo(
    const int inputNumberAgent,
    const int inputNumberShelter)
    :
    numberAgent(inputNumberAgent),
    numberShelter(inputNumberShelter)
  {}

  KnapsackInfo()
    :
    numberAgent(0),
    numberShelter(0)
  {}

  void addAgent(){ numberAgent++; }
  void addShelter(){ numberShelter++; }
  void setAgent(int inputAgent){ numberAgent = inputAgent; }
  void setShelter(int inputShelter){ numberShelter = inputShelter; }
  int getNumberAgent(){ return numberAgent; }
  int getNumberShelter(){ return numberShelter; }
};


void ShelterMergeSort(EscapeAgentInfo* escapeAgentInfo, KnapsackInfo knapsackInfo, ShelterInfo* shelterInfo);
void MergeSort(vector<int>* x, int left, int right, vector<int>* temp);

void knapasckSimple(ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo, KnapsackInfo knapsackInfo);
double DecideShelter(ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo, int* decidedAgentId, KnapsackInfo knapsackInfo);


//umeki add above-------------



class AgentProfileAndTaskTable {
public:
    AgentProfileAndTaskTable(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const AgentResource& masterResource,
        const size_t numberThreads);

    void CompleteInitialize(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const AgentResource& masterResource,
        const set<AgentIdType>& entireAgentIds);


    shared_ptr<AgentProfile> GetProfile(const string& profileName) const;

    shared_ptr<AgentTaskTable> GetTaskTable(
        const string& taskTableName,
        const string& profileName) const;

    shared_ptr<AgentTaskTable> GetEmptyTaskTable() const { return emptyTaskTablePtr; }

    bool ContainsTask(
        const string& taskTableName,
        const string& profileName) const;

    vector<string> GetProfileTypeNames() const;

    void GetLocationId(
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const LocationInfo& locationInfo,
        const bool ignoreLastPositionFromCandidate,
        const bool searchIntersection,
        GisPositionIdType& positionId,
        bool& isMultiplePositions,
        AgentResource& resource) const;
    
    //Umeki add
    /*void SettingAgentParam(
        const AgentResource& resource,
        const LocationInfo& locationInfo,
        GisPositionIdType& positionid,
        vector<GisPositionIdType> locationCandidateIds,
        int& agentFlag) const;
    */
    void SettingAgentParam(
        const AgentResource& resource,
        bool notObeyFlag) const;
    
    void SettingShelParam(
        const GisSubsystem& subsystem,
        vector<GisPositionIdType> locationCandidateIds,
        ShelterInfo* shelterInfo,
        EscapeAgentInfo* escapeAgentInfo,
        KnapsackInfo knapsackInfo,
        bool fileFlag) const;
    
    
    void SettingKnapsackParam(
        const GisSubsystem& subsystem,
        vector<GisPositionIdType> locationCandidateIds,
        bool fileFlag) const;
    
    
    //void knapasckSimple(ShelterInfo& shelterInfo, EscapeAgentInfo& escapeAgentInfo, int n);
    //EscapeAgentInfo DecideShelter(float* splitShel, int* priorityList, ShelterInfo* shelterInfo, EscapeAgentInfo* escapeAgentInfo){
    //void DecideShelter(ShelterInfo& shelterInfo, EscapeAgentInfo& escapeAgentInfo, int& decidedAgentId, int n);
    //Umeki add above

    AgentValueFormula MakeValueFormula(
        const AgentProfileType& profileType,
        const string& formula);

private:

    void LoadProfile(
        const GisSubsystem& theGisSubsystem,
        const AgentResource& masterResource,
        const string& profileFilePath,
        const double startTimeSec);

    void LoadTaskTable(
        const GisSubsystem& theGisSubsystem,
        const AgentResource& masterResource,
        const string& behaviorFilePath,
        const double startTimeSec,
        const map<string, set<string> >& availableTaskTables);

    void AddTask(
        const GisSubsystem& theGisSubsystem,
        const string& taskTableName,
        const AgentResource& masterResource,
        const string& taskLine,
        const double startTimeSec,
        const vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > >& defaultProfileTaskTablePtrs,
        const map<string, vector<GisPositionIdType> >& locationGroups);

    shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr;

    LabelMap<AgentProfileType, shared_ptr<AgentProfile> > profilePtrs;
    map<pair<string, AgentProfileType>, shared_ptr<AgentTaskTable> > taskTablePtrs;
    shared_ptr<AgentTaskTable> emptyTaskTablePtr;

    map<string, AgentCharactorIdType> charactorIds;

    map<string, vector<GisPositionIdType> > locationGroups;
    
    
};


class PhysicalDataReceiver: public SensingModel::DataReceiveHandler {
public:
    PhysicalDataReceiver(const AgentResource& initResource)
        :
        resource(initResource)
    {}

    virtual void ReceiveData(SensingSharedInfoType broadcastData) {
        resource.ReceivePhysicalData(broadcastData);
    }

private:
    AgentResource resource;
};

//--------------------------------------------------------------
// Multi Agent Simulator
//--------------------------------------------------------------

class PreSynchronizer {
public:
    virtual ~PreSynchronizer() {}
    virtual void Synchronize() const = 0;
};




class MultiAgentSimulator : public NetworkSimulator {
public:
    static const bool isDebugMode;

    MultiAgentSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const bool initRunSequentially);

    ~MultiAgentSimulator();

    void RunSimulationUntil(const TimeType& snapshotTime);

    TimeType CurrentTime() const { return currentTime; }
    TimeType NextTime() const { return currentTime + timeStep; }
    TimeType TimeStep() const { return timeStep; }

    TimeType GetTaskStartTime(const AgentResource& resource) const;

    RandomNumberGeneratorSeedType GetMobilitySeed() const { return mobilitySeed; }
    uint8_t GetCurrentSnapshotId() const { return (uint8_t)(currentSnapshotId); }
    uint8_t GetNextSnapshotId() const { return (currentSnapshotId+1)%NUMBER_TIMESTEP_SNAPSHOTS; }
    uint8_t GetPrevSnapshotId() const {
        assert(NUMBER_TIMESTEP_SNAPSHOTS == 2);
        return (currentSnapshotId+1)%NUMBER_TIMESTEP_SNAPSHOTS;
    }

    const ParameterDatabaseReader& GetParameterDatabaseReader() const {
        return *theParameterDatabaseReaderPtr;
    }

    void AddVehicle(const shared_ptr<Vehicle>& vehiclePtr);
    void DeleteVehicle(const shared_ptr<Vehicle>& vehiclePtr);

    shared_ptr<AgentValueFormula> CreateValueFormula(
        const LabelMap<AgentStatusIdType, AgentValueFormula>& statusValues,
        const string& aString) const;

    shared_ptr<PublicVehicleTable> GetPublicVehicleTable() const { return thePublicVehicleTablePtr; }

    AgentResource GetMasterBusAgentResource() { return AgentResource(masterBusAgentPtr); }
    AgentResource GetMasterTaxiAgentResource() { return AgentResource(masterTaxiAgentPtr); }

    void AttachCommunicationNode(
        const AgentIdType& agentId,
        const shared_ptr<AgentCommunicationNode>& communicationNodePtr);

    void DetachCommunicationNode(
        const AgentIdType& agentId,
        const shared_ptr<AgentCommunicationNode>& communicationNodePtr);

    void OutputAgentTraces();

    void CreateApplicationForNode(
        const AgentResource& resource,
        const NodeIdType& sourceNodeId,
        const InterfaceOrInstanceIdType& instanceId,
        const vector<string>& parameterLines,
        const set<NodeIdType>& targetNodeIds);

    void OutputTraceEvent(
        const NodeIdType& gisObjectOrAgentId,
        const string& eventName,
        const double value) const;

    void OutputStringTraceEvent(
        const NodeIdType& gisObjectOrAgentId,
        const string& eventName,
        const string& value) const;

    void AddCommunicationNode(
        const shared_ptr<AgentCommunicationNode>& aNodePtr);

    shared_ptr<AgentTaskTable> GetTaskTable(
        const string& taskTableName,
        const string& profileName) const { return theProfileAndTaskTable.GetTaskTable(taskTableName, profileName); }

    void SetOwnerAgent(
        const AgentResource& resource,
        const AgentIdType& ownerAgentId = MASTER_ANY_AGENT_ID);

    void RemoveOwnerAgent(const AgentResource& resource) { (*this).SetOwnerAgent(resource); }

    void RemoveOwnerAgentChange(const AgentResource& resource);

    void SyncGisTopology() { (*this).ExecuteTimestepBasedEvent(); }

    void AddTaxi(const shared_ptr<Taxi>& taxiPtr);

    void CreateCommunicationNodeAtWakeupTimeFor(const NodeIdType& agentId) {
        createCommunicationNodeAtWakeupTimeFor.insert(agentId);
    }

    void OutputTrace(const string& aString) const;

    void RecordAssignedProfileValuesToFile(
        const AgentIdType& agentId,
        const shared_ptr<AgentProfile>& profilePtr,
        const map<string, double>& assignedValues,
        const bool hasCar,
        const bool hasBicycle) {

        profileValueOutputSubsystem.RecordAssignedProfileValuesToFile(agentId, profilePtr, assignedValues, hasCar, hasBicycle);
    }

protected:
    map<NodeIdType, TimeType> wakeupTimes;

    bool IsEqualToAgentId(const NodeIdType& nodeId) const;
    TimeType GetWakeupTime(const NodeIdType& nodeId) const;
    bool SupportMultiAgent() const { return true; }

private:

    void AddAgent(const shared_ptr<Agent>& agentPtr, const bool withNodeGeneration = true);
    void AddAgentWithoutNodeGeneration(const shared_ptr<Agent>& agentPtr) {
        (*this).AddAgent(agentPtr, false);
    }

    shared_ptr<VehicleNode> CreateVehicleNode(
        const AgentIdType& agentId,
        const VehicleType& vehicleType);

    shared_ptr<SimulationEngineInterface> GetSimEngineInterfacePtr(const AgentIdType& agentId);

    void ReserveVehicleNode(
        const AgentIdType& agentId,
        const VehicleType& vehicleType);

    void ChangeAgentOwnerIfNecessary();
    void DeleteInactiveAgents();
    void AddVehicle();
    void AddTrain(const vector<shared_ptr<Train> >& trainPtrs);
    void AddBus(const vector<shared_ptr<Bus> >& busPtrs);
    void AddTaxi(const vector<pair<RailRoadStationIdType, AgentResource> >& calledTaxis);
    void AddCar();

    void IncrementTimeStep();
    void IncrementTimeStep(const size_t threadNumber);

    void TakePreSynchronizer(
        shared_ptr<PreSynchronizer>& preSynchronizerPtr,
        bool& success);

    void ReturnPreSynchronizer();

    void RerouteAllAgents();

    void AdvanceTimeStep();

    class AdvanceTimeStepEvent : public SimulationEvent {
    public:
        AdvanceTimeStepEvent(MultiAgentSimulator* initSimulatorPtr) : simulatorPtr(initSimulatorPtr) {}
        void ExecuteEvent() { simulatorPtr->AdvanceTimeStep(); }
    private:
        MultiAgentSimulator* simulatorPtr;
    };

    class TimeIncrementThreadFunctor {
    public:
        TimeIncrementThreadFunctor(
            MultiAgentSimulator* initSimulatorPtr,
            const size_t initThreadNumber)
            :
            simulatorPtr(initSimulatorPtr),
            threadNumber(initThreadNumber)
        {}
        void operator()() { simulatorPtr->IncrementTimeStep(threadNumber); }
    private:
        MultiAgentSimulator* simulatorPtr;
        size_t threadNumber;
    };

    size_t currentSnapshotId;
    TimeType currentTime;
    TimeType timeStep;
    bool isSimulationDone;

    const unsigned int numberThreads;
    static const string modelName;

    struct OwnerChangeEvent {
        AgentIdType agentId;
        AgentIdType ownerId;

        OwnerChangeEvent()
            :
            agentId(MASTER_ANY_AGENT_ID),
            ownerId(MASTER_ANY_AGENT_ID)
        {}

        OwnerChangeEvent(
            const AgentIdType& initAgentId,
            const AgentIdType& initOwnerId)
            :
            agentId(initAgentId),
            ownerId(initOwnerId)
        {}
    };

    struct DynamicApplucationData {
        NodeIdType sourceNodeId;
        InterfaceOrInstanceIdType instanceId;
        vector<string> parameterLines;
        set<NodeIdType> targetNodeIds;

        DynamicApplucationData(
            const NodeIdType& initSourceNodeId,
            const InterfaceOrInstanceIdType& initInstanceId,
            const vector<string>& initParameterLines,
            const set<NodeIdType>& initTargetNodeIds)
            :
            sourceNodeId(initSourceNodeId),
            instanceId(initInstanceId),
            parameterLines(initParameterLines),
            targetNodeIds(initTargetNodeIds)
        {}
    };

    void SyncApplicationCreation();
    void GenerateDynamicApplication(const DynamicApplucationData& dynamicApplicationData);

    class ApplicationCreationSynchronizer : public PreSynchronizer {
    public:
        ApplicationCreationSynchronizer(MultiAgentSimulator* initSimulatorPtr)
            :
            simulatorPtr(initSimulatorPtr)
        {}
        virtual void Synchronize() const {
            simulatorPtr->SyncApplicationCreation();
        }
    private:
        MultiAgentSimulator* simulatorPtr;
    };

    class ProfileValueOutputSubsystem {
    public:
        ProfileValueOutputSubsystem(
            const ParameterDatabaseReader& initParameterDatabaseReader);

        void RecordAssignedProfileValuesToFile(
            const AgentIdType& agentId,
            const shared_ptr<AgentProfile>& profilePtr,
            const map<string, double>& assignedValues,
            const bool hasCar,
            const bool hasBicycle);

    private:
        ofstream profileValueOutputFile;
    };

    ProfileValueOutputSubsystem profileValueOutputSubsystem;

    struct ThreadPartition {
        shared_ptr<boost::thread> timeIncrementThreadPtr;
        queue<AgentIdType> deleteNodeIds;
        list<shared_ptr<Agent> > agentPtrs;
        queue<shared_ptr<PreSynchronizer> > tookSynchronizerPtrs;
        map<AgentIdType, list<shared_ptr<Agent> >::iterator> iterToAgentList;

        list<shared_ptr<Vehicle> > newlyAddedVehiclePtrs;
        map<AgentIdType, OwnerChangeEvent> ownerChangeEvents;
        vector<DynamicApplucationData> dynamicApplicationDatas;

        ThreadPartition() {}

        ~ThreadPartition();

        static const int SEED_HASHING_INPUT = 3520163;
    };
    vector<ThreadPartition> threadPartitions;
    boost::barrier timeIncrementThreadBarrier;
    boost::mutex preSynchronizerMutex;
    queue<shared_ptr<PreSynchronizer> > preSynchronizerPtrs;

    vector<queue<shared_ptr<VehicleNode> > > reservedVehicleNodePtrs;

    shared_ptr<MultiAgentGis> theAgentGisPtr;

    shared_ptr<Agent> masterAnyAgentPtr;
    shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr;
    AgentProfileAndTaskTable theProfileAndTaskTable;

    shared_ptr<Agent> masterBusAgentPtr;
    shared_ptr<Agent> masterTaxiAgentPtr;

    shared_ptr<AgentRouteSearchSubsystem> theRouteSearchSubsystemPtr;

    shared_ptr<SimulationEngineInterface> theSimulationRunInterfacePtr;

    struct AgentWakeupEntry {
        TimeType time;
        shared_ptr<Agent> agentPtr;

        AgentWakeupEntry()
            :
            time(),
            agentPtr()
        {}

        AgentWakeupEntry(
            const TimeType& initTime,
            const shared_ptr<Agent>& initAgentPtr)
            :
            time(initTime),
            agentPtr(initAgentPtr)
        {}

        bool operator<(const AgentWakeupEntry& right) const {
            return (time > right.time);
        }
    };

    set<AgentIdType> entireAgentIds;
    priority_queue_stable<AgentWakeupEntry> agentWakeupQueue;

    set<AgentIdType> newlyAddedAgentIds;

    map<AgentIdType, list<shared_ptr<AgentCommunicationNode> > > communicationNodePtrsWaitingAgentCreation;
    map<AgentIdType, shared_ptr<AgentCommunicationNode> > synchronizedNodePtrs;
    map<AgentIdType, size_t> agentThreadNumbers;
    set<AgentIdType> createCommunicationNodeAtWakeupTimeFor;

    class MultiAgentGisChangeEventHandler : public GisSubsystem::GisChangeEventHandler {
    public:
        MultiAgentGisChangeEventHandler(
            MultiAgentSimulator* initSimulator) : simulator(initSimulator)
        {}

        virtual void GisInformationChanged() {
            simulator->RerouteAllAgents();
        }
    private:
        MultiAgentSimulator* simulator;
    };
};//MultiagentSimulator//


inline
void MultiAgentSimulator::OutputTrace(const string& aString) const
{
    if (isDebugMode) {
        const TimeType time = (*this).CurrentTime();
        cout << ConvertTimeToStringSecs(time) << ":[engine] " << aString << endl;
    }//if//
}


// referable variable is frip(cached) variable for inter-thread access.
// Note: A value revision is applyed to next time step.
// You should update these variables whenever a time step event is executed.

template <typename T>
class ReferableVariable {
public:
    explicit ReferableVariable(const MultiAgentSimulator* initSimulatorPtr)
        :
        simulatorPtr(initSimulatorPtr),
        updatedFlag(false),
        unchangedCount(0)
    {}

    bool IsChanged() const { return (unchangedCount == 0); }
    void Unchanged() {
        unchangedCount++;

        if (unchangedCount < NUMBER_TIMESTEP_SNAPSHOTS) {
            values[simulatorPtr->GetNextSnapshotId()] =
                values[simulatorPtr->GetCurrentSnapshotId()];
        } else {
            unchangedCount = NUMBER_TIMESTEP_SNAPSHOTS;
        }

        updatedFlag = true;
    }

    const T& operator*() const { return values[simulatorPtr->GetCurrentSnapshotId()]; }
    const T& GetValue() const { return *(*this); }
    const T& GetNextValue() const { return values[simulatorPtr->GetNextSnapshotId()]; }
    const T& GetPrevValue() const { return values[simulatorPtr->GetPrevSnapshotId()]; }

    void Initialize(const T& aValue) {
        for(size_t i = 0; i < NUMBER_TIMESTEP_SNAPSHOTS; i++) {
            values[i] = aValue;
        }
    }

    void SetValue(const T& aValue) {
        unchangedCount = 0;
        values[simulatorPtr->GetNextSnapshotId()] = aValue;
        updatedFlag = true;
    }

    void UnsetUpdateFlag() { updatedFlag = false; }

    void UpdateWithPrevValueIfNotUpdate() {
        if (!updatedFlag) {
            (*this).Unchanged();
        }
    }

private:
    const MultiAgentSimulator* simulatorPtr;
    T values[NUMBER_TIMESTEP_SNAPSHOTS];

    bool updatedFlag;
    uint8_t unchangedCount;
};

inline
void Agent::OutputTrace(const string& aString) const
{
    if (MultiAgentSimulator::isDebugMode) {
        const TimeType time = simulatorPtr->CurrentTime();
        cout << ConvertTimeToStringSecs(time) << ":[" <<  std::setw(4)
             << agentId << "(v" << lastVertexId << ")] " << aString << endl;
    }//if//
}



struct shelterList {
    int count;
    float escapeTime[MAX_S];
};



}//namespace

#endif
