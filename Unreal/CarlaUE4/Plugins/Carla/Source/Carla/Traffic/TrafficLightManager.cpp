// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "TrafficLightManager.h"
#include "Game/CarlaStatics.h"
#include "Components/BoxComponent.h"
#include <string>

ATrafficLightManager::ATrafficLightManager()
{
  PrimaryActorTick.bCanEverTick = false;
  SceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
  RootComponent = SceneComponent;

  // Hard codded default traffic light blueprint
  static ConstructorHelpers::FObjectFinder<UBlueprint> TrafficLightFinder(
      TEXT( "Blueprint'/Game/Carla/Blueprints/TrafficLight/BP_TLOpenDrive.BP_TLOpenDrive'" ) );
  if (TrafficLightFinder.Succeeded())
  {
    TrafficLightModel = TrafficLightFinder.Object->GeneratedClass;
  }
}

void ATrafficLightManager::RegisterLightComponent(UTrafficLightComponent * TrafficLight)
{
  // Cast to std::string
  carla::road::SignId SignId(TCHAR_TO_UTF8(*(TrafficLight->GetSignId())));

  // Get OpenDRIVE signal
  if (GetMap()->GetSignals().count(SignId) == 0)
  {
    return;
  }
  const auto &Signal = GetMap()->GetSignals().at(SignId);
  if(Signal->GetControllers().empty())
  {
    return;
  }
  // Only one controller per signal
  auto ControllerId = *(Signal->GetControllers().begin());

  // Get controller
  const auto &Controller = GetMap()->GetControllers().at(ControllerId);
  if(Controller->GetJunctions().empty())
  {
    return;
  }
  // Get junction of the controller
  auto JunctionId = *(Controller->GetJunctions().begin());

  // Search/create TrafficGroup (junction traffic light manager)
  if(!TrafficGroups.Contains(JunctionId))
  {
    auto * TrafficLightGroup =
        GetWorld()->SpawnActor<ATrafficLightGroup>();
    TrafficLightGroup->JunctionId = JunctionId;
    TrafficGroups.Add(JunctionId, TrafficLightGroup);
  }
  auto * TrafficLightGroup = TrafficGroups[JunctionId];

  // Search/create controller in the junction
  if(!TrafficControllers.Contains(ControllerId.c_str()))
  {
    auto *TrafficLightController = NewObject<UTrafficLightController>();
    TrafficLightController->SetControllerId(ControllerId.c_str());
    TrafficLightGroup->GetControllers().Add(TrafficLightController);
    TrafficControllers.Add(ControllerId.c_str(), TrafficLightController);
  }
  auto *TrafficLightController = TrafficControllers[ControllerId.c_str()];

  TrafficLight->TrafficLightGroup = TrafficLightGroup;

  // Add signal to controller
  TrafficLightController->AddTrafficLight(TrafficLight);
  TrafficLightController->ResetState();

  // Add signal to map
  TrafficLights.Add(TrafficLight->GetSignId(), TrafficLight);

  TrafficLightGroup->ResetGroup();
}

const boost::optional<carla::road::Map>& ATrafficLightManager::GetMap()
{
  if (!Map.has_value())
  {
    FString MapName = GetWorld()->GetName();
    std::string opendrive_xml = carla::rpc::FromFString(UOpenDrive::LoadXODR(MapName));
    Map = carla::opendrive::OpenDriveParser::Load(opendrive_xml);
    if (!Map.has_value()) {
      UE_LOG(LogCarla, Error, TEXT("Invalid Map"));
    }
  }
  return Map;
}

void ATrafficLightManager::GenerateTrafficLights()
{
  if(!TrafficLightModel)
  {
    return;
  }

  RemoveExistingTrafficLights();

  SpawnTrafficLights();

  GenerateTriggerBoxes();
}

// Called when the game starts
void ATrafficLightManager::BeginPlay()
{
  Super::BeginPlay();
}

ATrafficLightGroup* ATrafficLightManager::GetTrafficGroup(
    carla::road::JuncId JunctionId)
{
  if (TrafficGroups.Contains(JunctionId))
  {
    return TrafficGroups[JunctionId];
  }
  return nullptr;
}


UTrafficLightController* ATrafficLightManager::GetController(
    FString ControllerId)
{
  if (TrafficControllers.Contains(ControllerId))
  {
    return TrafficControllers[ControllerId];
  }
  return nullptr;
}

UTrafficLightComponent* ATrafficLightManager::GetTrafficLight(FString SignId)
{
  if (!TrafficLights.Contains(SignId))
  {
    return nullptr;
  }
  return TrafficLights[SignId];
}

void ATrafficLightManager::RemoveExistingTrafficLights()
{
  for (auto GroupPair : TrafficGroups)
  {
    auto *TrafficLightGroup = GroupPair.Value;
    TrafficLightGroup->Destroy();
  }

  for (auto LightPair : TrafficLights)
  {
    auto *TrafficLightComponent = LightPair.Value;
    TrafficLightComponent->GetOwner()->Destroy();
  }

  TrafficGroups.Empty();
  TrafficControllers.Empty();
  TrafficLights.Empty();
}

void ATrafficLightManager::SpawnTrafficLights()
{
    const auto &Signals = GetMap()->GetSignals();
  for(const auto& ControllerPair : GetMap()->GetControllers())
  {
    const auto& Controller = ControllerPair.second;
    for(const auto& SignalId : Controller->GetSignals())
    {
      const auto& Signal = Signals.at(SignalId);
      auto CarlaTransform = Signal->GetTransform();
      FTransform SpawnTransform(CarlaTransform);

      FVector SpawnLocation = SpawnTransform.GetLocation();
      FRotator SpawnRotation(SpawnTransform.GetRotation());
      SpawnRotation.Yaw += 90;

      FActorSpawnParameters SpawnParams;
      SpawnParams.Owner = this;
      SpawnParams.SpawnCollisionHandlingOverride =
          ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      AActor * TrafficLight = GetWorld()->SpawnActor<AActor>(
          TrafficLightModel,
          SpawnLocation,
          SpawnRotation,
          SpawnParams);

      UTrafficLightComponent *TrafficLightComponent =
          NewObject<UTrafficLightComponent>(TrafficLight);
      TrafficLightComponent->SetSignId(SignalId.c_str());
      TrafficLightComponent->RegisterComponent();
      TrafficLightComponent->AttachToComponent(
          TrafficLight->GetRootComponent(),
          FAttachmentTransformRules::KeepRelativeTransform);
    }
  }
}

std::vector<int> GenerateRange(int a, int b)
{
  std::vector<int> result;
  if (a < b)
  {
    for(int i = a; i <= b; ++i)
    {
      result.push_back(i);
    }
  }
  else
  {
    for(int i = a; i >= b; --i)
    {
      result.push_back(i);
    }
  }
  return result;
}

void ATrafficLightManager::GenerateTriggerBox(
    carla::road::element::Waypoint &waypoint,
    UTrafficLightComponent* TrafficLightComponent,
    float BoxSize)
{
  // convert from m to cm
  float UEBoxSize = 100 * BoxSize;
  AActor *ParentActor = TrafficLightComponent->GetOwner();
  FTransform ReferenceTransform = GetMap()->ComputeTransform(waypoint);
  UBoxComponent *BoxComponent = NewObject<UBoxComponent>(ParentActor);
  BoxComponent->RegisterComponent();
  BoxComponent->AttachToComponent(
      ParentActor->GetRootComponent(),
      FAttachmentTransformRules::KeepRelativeTransform);
  BoxComponent->SetWorldTransform(ReferenceTransform);
  BoxComponent->OnComponentBeginOverlap.AddDynamic(TrafficLightComponent,
      &USignComponent::OnOverlapBeginInterface);
  BoxComponent->SetBoxExtent(FVector(UEBoxSize, UEBoxSize, UEBoxSize), true);

  // Debug
  // DrawDebugBox(GetWorld(),
  //     ReferenceTransform.GetLocation(),
  //     BoxComponent->GetScaledBoxExtent(),
  //     ReferenceTransform.GetRotation(),
  //     FColor(0, 0, 200),
  //     true);
}

void ATrafficLightManager::GenerateTriggerBoxes()
{
  const double epsilon = 0.00001;

  // Spawn trigger boxes
  auto waypoints = GetMap()->GenerateWaypointsOnRoadEntries();
  std::unordered_set<carla::road::RoadId> ExploredRoads;
  for (auto & waypoint : waypoints)
  {
    // Check if we alredy explored this road
    if (ExploredRoads.count(waypoint.road_id) > 0)
    {
      continue;
    }
    ExploredRoads.insert(waypoint.road_id);

    // Multiple times for same road (performance impact, not in behavior)
    auto SignalReferences = GetMap()->GetLane(waypoint).
        GetRoad()->GetInfos<carla::road::element::RoadInfoSignal>();
    for (auto *SignalReference : SignalReferences)
    {
      FString SignalId(SignalReference->GetSignalId().c_str());
      if(TrafficLights.Contains(SignalId))
      {
        auto *TrafficLightComponent = TrafficLights[SignalId];
        for(auto &validity : SignalReference->GetValidities())
        {
          for(auto lane : GenerateRange(validity._from_lane, validity._to_lane))
          {
            if(lane == 0)
              continue;

            auto signal_waypoint = GetMap()->GetWaypoint(
                waypoint.road_id, lane, SignalReference->GetS()).get();

            // Get 90% of the half size of the width of the lane
            float BoxSize = static_cast<float>(
                0.9*GetMap()->GetLaneWidth(waypoint)/2.0);
            // Get min and max
            double LaneLength = GetMap()->GetLane(signal_waypoint).GetLength();
            double LaneDistance = GetMap()->GetLane(signal_waypoint).GetDistance();
            if(lane < 0)
            {
              signal_waypoint.s = FMath::Clamp(signal_waypoint.s - BoxSize,
                  LaneDistance + epsilon, LaneDistance + LaneLength - epsilon);
            }
            else
            {
              signal_waypoint.s = FMath::Clamp(signal_waypoint.s + BoxSize,
                  LaneDistance + epsilon, LaneDistance + LaneLength - epsilon);
            }
            GenerateTriggerBox(signal_waypoint, TrafficLightComponent, BoxSize);
          }
        }
      }
    }
  }
}
