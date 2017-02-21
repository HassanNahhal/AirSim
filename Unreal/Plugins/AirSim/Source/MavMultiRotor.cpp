#include "AirSim.h"
#include "MavMultiRotor.h"
#include "AirBlueprintLib.h"
#include "control/Settings.h"

using namespace msr::airlib;

void MavMultiRotor::initialize(AFlyingPawn* vehicle_pawn)
{
	vehicle_pawn_ = vehicle_pawn;
	vehicle_pawn_->initialize();

	//init vehicle
	auto initial_kinematics = Kinematics::State::zero();
	initial_kinematics.pose = vehicle_pawn_->getPose();
	msr::airlib::Environment::State initial_environment;
	initial_environment.position = initial_kinematics.pose.position;
	initial_environment.geo_point = vehicle_pawn_->getHomePoint();
	initial_environment.min_z_over_ground = vehicle_pawn_->getMinZOverGround();
	environment_.initialize(initial_environment);
    createController(vehicle_);
	vehicle_.initialize(Px4QuadX::Params(), initial_kinematics,
		&environment_, controller_.get());
}

void MavMultiRotor::createController(MultiRotor& vehicle)
{
    controller_.reset(new MavLinkHelper());
    auto mav_controller = static_cast<MavLinkHelper*>(controller_.get());
    mav_controller->initialize(getConnectionInfo(), &vehicle);
}

void MavMultiRotor::beginPlay()
{
    //connect to HIL
    try {

        controller_->start();
    }
    catch (std::exception ex) {

        UAirBlueprintLib::LogMessage(FString("Vehicle controller cannot be started, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 180);
    }
}

void MavMultiRotor::endPlay()
{
    controller_->stop();
}

msr::airlib::MavLinkHelper::HILConnectionInfo MavMultiRotor::getConnectionInfo()
{
	auto connection_info = vehicle_pawn_->getHILConnectionInfo();

	Settings& settings = Settings::singleton();

	auto settings_filename = Settings::singleton().getFileName();
	if (!settings_filename.empty()) {
		Settings child = settings.getChild(connection_info.vehicle_name);
		// allow json overrides on a per-vehicle basis.
		connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
		connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
		connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
		connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
		connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
	}

	if (connection_info.vehicle_name.size() > 0) {
		Settings child;
		child.setBool("UseSerial", connection_info.use_serial);
		child.setString("UdpIp", connection_info.ip_address);
		child.setInt("UdpPort", connection_info.ip_port);
		child.setString("SerialPort", connection_info.serial_port);
		child.setInt("SerialBaudRate", connection_info.baud_rate);
		settings.setChild(connection_info.vehicle_name, child);
		settings.saveJSonFile(L"settings.json");
	}

	return connection_info;
}

void MavMultiRotor::updateRenderedState()
{
	//move collison info from rendering engine to vehicle
	vehicle_.setCollisionInfo(vehicle_pawn_->getCollisonInfo());
	//update ground level
	environment_.getState().min_z_over_ground = vehicle_pawn_->getMinZOverGround();
	//update pose of object in rendering engine
	vehicle_pawn_->setPose(vehicle_.getPose());

	//update rotor poses
	for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
		const auto& rotor_output = vehicle_.getRotorOutput(i);
		rotor_speeds_[i] = rotor_output.speed;
		rotor_directions_[i] = rotor_output.turning_direction;
		rotor_thrusts_[i] = rotor_output.thrust;
		rotor_controls_filtered_[i] = rotor_output.control_signal_filtered;
	}

	//retrieve MavLink status messages
    controller_->getStatusMessages(controller_messages_);
}

void MavMultiRotor::updateRendering()
{
	//update rotor animations
	for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
		vehicle_pawn_->setRotorSpeed(i, rotor_speeds_[i] * rotor_directions_[i]);
	}

	for (auto i = 0; i < controller_messages_.size(); ++i) {
		UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
	}
}


//*** Start: UpdatableState implementation ***//
void MavMultiRotor::reset()
{
	vehicle_pawn_->reset();    //we do flier resetPose so that flier is placed back without collisons
	vehicle_.reset();
}

void MavMultiRotor::update(real_T dt)
{
	//this is high frequency physics tick, flier gets ticked at rendering frame rate
	vehicle_.update(dt);
}

void MavMultiRotor::reportState(StateReporter& reporter)
{
	vehicle_.reportState(reporter);
}

MavMultiRotor::UpdatableObject* MavMultiRotor::getPhysicsBody()
{
	return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

msr::airlib::DroneControlBase* MavMultiRotor::createOrGetDroneControl()
{
	return controller_->createOrGetDroneControl();
}
