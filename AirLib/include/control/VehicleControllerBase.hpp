// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_VehicleControllerBase_hpp
#define msr_air_copter_sim_VehicleControllerBase_hpp

#include "control/ControllerBase.hpp"

namespace msr { namespace airlib {

/*
    Defined additional interface for vehicles
*/
class VehicleControllerBase : public ControllerBase {
public:
    //tells the controller to accept simulated sensor inputs
    virtual void setSimulationMode(bool is_set) = 0;
    
    //tells the controller to switch from human operated mode to computer operated mode
    virtual void setOffboardMode(bool is_set) = 0;

    //Supplies the controller 16 channels of input
    virtual void setManualUserInputs(float inputs[], size_t count)
    {
        //default implementation
    }
};

}} //namespace
#endif
