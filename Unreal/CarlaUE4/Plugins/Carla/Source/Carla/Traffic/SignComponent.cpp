// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "SignComponent.h"

USignComponent::USignComponent()
{
  PrimaryComponentTick.bCanEverTick = false;
}

// Called when the game starts
void USignComponent::BeginPlay()
{
  Super::BeginPlay();

}

void USignComponent::OnOverlapBeginInterface(UPrimitiveComponent *OverlappedComp,
    AActor *OtherActor,
    UPrimitiveComponent *OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult &SweepResult)
{
  OnOverlapBegin(OverlappedComp,
      OtherActor,
      OtherComp,
      OtherBodyIndex,
      bFromSweep,
      SweepResult);
}

void USignComponent::OnOverlapEndInterface(UPrimitiveComponent *OverlappedComp,
    AActor *OtherActor,
    UPrimitiveComponent *OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult &SweepResult)
{
  OnOverlapEnd(OverlappedComp,
      OtherActor,
      OtherComp,
      OtherBodyIndex,
      bFromSweep,
      SweepResult);
}

void USignComponent::OnOverlapBegin(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult)
{
  ACarlaWheeledVehicle * Vehicle = Cast<ACarlaWheeledVehicle>(OtherActor);
  if (Vehicle)
  {
    AWheeledVehicleAIController* VehicleController =
        Cast<AWheeledVehicleAIController>(Vehicle->GetController());
    if (VehicleController)
    {
      // VehicleController->SetTrafficLightState(TrafficSignState);
    }
  }
}

void USignComponent::OnOverlapEnd(UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult)
{
  ACarlaWheeledVehicle * Vehicle = Cast<ACarlaWheeledVehicle>(OtherActor);
  if (Vehicle)
  {
    AWheeledVehicleAIController* VehicleController =
        Cast<AWheeledVehicleAIController>(Vehicle->GetController());
    if (VehicleController)
    {
      // VehicleController->SetTrafficLightState(TrafficSignState);
    }
  }
}

// Called every frame
void USignComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

const FString& USignComponent::GetSignId() const
{
  return SignId;
}

void USignComponent::SetSignId(const FString &Id) {
  SignId = Id;
}

void USignComponent::SetTrafficSignState(ETrafficSignState State) {
  TrafficSignState = State;
}
