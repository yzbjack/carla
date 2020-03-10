// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Carla/OpenDrive/OpenDrive.h"
#include "TrafficSignBase.h"
#include "SignComponent.generated.h"

/// Class representing an OpenDRIVE Signal
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CARLA_API USignComponent : public USceneComponent
{
  GENERATED_BODY()

public:
  USignComponent();

  UFUNCTION(BlueprintPure)
  const FString &GetSignId() const;

  UFUNCTION(BlueprintCallable)
  void SetSignId(const FString &Id);

  UFUNCTION(BlueprintCallable)
  void SetTrafficSignState(ETrafficSignState State);

  UFUNCTION(BlueprintCallable)
  void OnOverlapBeginInterface(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult);

  void OnOverlapEndInterface(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult);

protected:
  // Called when the game starts
  virtual void BeginPlay() override;

  // Called every frame
  virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

  virtual void OnOverlapBegin(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult);

  virtual void OnOverlapEnd(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult);

  ETrafficSignState TrafficSignState;

private:

  UPROPERTY(Category = "Traffic Sign", EditAnywhere)
  FString SignId;

};
