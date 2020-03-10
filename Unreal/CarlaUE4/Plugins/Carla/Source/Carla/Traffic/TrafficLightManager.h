// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "TrafficLightComponent.h"
#include "TrafficLightGroup.h"
#include "Game/CarlaGameModeBase.h"
#include "Carla/OpenDrive/OpenDrive.h"
#include "TrafficLightManager.generated.h"

/// Class In charge of creating and assigning traffic
/// light groups, controllers and components.
/// It creates traffic signals as well (stop, yields, etc)
UCLASS()
class CARLA_API ATrafficLightManager : public AActor
{
  GENERATED_BODY()

public:

  ATrafficLightManager();

  UFUNCTION(BlueprintCallable)
  void RegisterLightComponent(UTrafficLightComponent * TrafficLight);

  const boost::optional<carla::road::Map> &GetMap();

  UFUNCTION(BlueprintCallable)
  ATrafficLightGroup* GetTrafficGroup(int JunctionId);

  UFUNCTION(BlueprintCallable)
  UTrafficLightController* GetController(FString ControllerId);

  UFUNCTION(BlueprintCallable)
  USignComponent* GetTrafficSign(FString SignId);

  UFUNCTION(CallInEditor)
  void GenerateSignalsAndTrafficLights();

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

private:

  // Cached Carla Game Mode
  UPROPERTY()
  ACarlaGameModeBase *GameMode = 0;

  // Mapped references to ATrafficLightGroup (junction)
  UPROPERTY()
  TMap<int, ATrafficLightGroup *> TrafficGroups;

  // Mapped references to UTrafficLightController (controllers)
  UPROPERTY()
  TMap<FString, UTrafficLightController *> TrafficControllers;

  // References to traffic signals
  UPROPERTY()
  TMap<FString, USignComponent *> TrafficSigns;

  UPROPERTY(EditAnywhere, Category= "Traffic Light Manager")
  TSubclassOf<AActor> TrafficLightModel;

  UPROPERTY(EditAnywhere, Category= "Traffic Light Manager")
  TMap<FString, TSubclassOf<AActor>> TrafficSignsModels;

  UPROPERTY(Category = "Traffic Light Manager", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
  USceneComponent *SceneComponent;

  boost::optional<carla::road::Map> Map;

  void RemoveExistingTrafficLights();

  void SpawnTrafficLights();

  void SpawnSignals();

  void GenerateTriggerBoxes();

  void GenerateTriggerBox(
      carla::road::element::Waypoint &waypoint,
      USignComponent* SignComponent,
      float BoxSize);
};
