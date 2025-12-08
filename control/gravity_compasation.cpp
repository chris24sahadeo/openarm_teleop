// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <controller/dynamics.hpp>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <linux/can.h>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <thread>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

void set_control_mode_all(openarm::can::socket::OpenArm* openarm, uint8_t mode) {
    std::cout << "Setting control mode to " << static_cast<int>(mode) << " for all motors..."
              << std::endl;

    // Access the CAN socket to send raw parameter write commands
    auto& master_collection = openarm->get_master_can_device_collection();
    auto& can_socket = master_collection.get_can_socket();

    // Set control mode for arm motors
    const auto& arm_motors = openarm->get_arm().get_motors();
    for (const auto& motor : arm_motors) {
        // Create CAN frame for parameter write command
        // Format matches Python encode_write_register_int:
        // struct.pack("<HBBI", slave_id, 0x55, register_address, value)
        can_frame frame;
        frame.can_id = 0x7FF;  // Master command ID (not individual motor slave ID)
        frame.can_dlc = 8;

        // Pack data in little-endian format matching Python struct
        uint16_t slave_id = motor.get_send_can_id();
        std::memcpy(&frame.data[0], &slave_id, sizeof(uint16_t));  // slave_id (H = 2 bytes)

        frame.data[2] = 0x55;  // Write register command (B = 1 byte)
        // Register address (B = 1 byte)
        frame.data[3] = static_cast<uint8_t>(openarm::damiao_motor::RID::CTRL_MODE);

        // Pack mode value as uint32 (I = 4 bytes) in little-endian
        uint32_t mode_uint32 = static_cast<uint32_t>(mode);
        std::memcpy(&frame.data[4], &mode_uint32, sizeof(uint32_t));

        can_socket.write_can_frame(frame);
    }

    // Set control mode for gripper
    const auto* gripper_motor = openarm->get_gripper().get_motor();
    if (gripper_motor) {
        can_frame frame;
        frame.can_id = 0x7FF;  // Master command ID
        frame.can_dlc = 8;

        uint16_t slave_id = gripper_motor->get_send_can_id();
        std::memcpy(&frame.data[0], &slave_id, sizeof(uint16_t));

        frame.data[2] = 0x55;  // Write register command
        frame.data[3] = static_cast<uint8_t>(openarm::damiao_motor::RID::CTRL_MODE);

        uint32_t mode_uint32 = static_cast<uint32_t>(mode);
        std::memcpy(&frame.data[4], &mode_uint32, sizeof(uint32_t));

        can_socket.write_can_frame(frame);
    }

    // Wait for commands to be processed and read responses
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    openarm->recv_all();

    std::cout << "Control mode set to " << static_cast<int>(mode) << " for all motors" << std::endl;
}

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string arm_side = "right_arm";
        std::string can_interface = "can0";

        if (argc < 4) {
            std::cerr << "Usage: " << argv[0] << " <arm_side> <can_interface> <urdf_path>"
                      << std::endl;
            std::cerr << "Example: " << argv[0] << " right_arm can0 /tmp/v10_bimanual.urdf"
                      << std::endl;
            return 1;
        }

        arm_side = argv[1];
        can_interface = argv[2];
        std::string urdf_path = argv[3];

        if (arm_side != "left_arm" && arm_side != "right_arm") {
            std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                      << ". Must be 'left_arm' or 'right_arm'." << std::endl;
            return 1;
        }

        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        std::cout << "=== OpenArm Gravity Compensation ===" << std::endl;
        std::cout << "Arm side       : " << arm_side << std::endl;
        std::cout << "CAN interface  : " << can_interface << std::endl;
        std::cout << "URDF path      : " << urdf_path << std::endl;

        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        Dynamics arm_dynamics(urdf_path, root_link, leaf_link);
        arm_dynamics.Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm* openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(can_interface, false);

        // Set MIT mode for all motors
        set_control_mode_all(openarm, 1);

        // Enable all motors
        openarm->enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm->recv_all();
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_hz_display = start_time;
        int frame_count = 0;

        std::vector<double> arm_joint_positions(openarm->get_arm().get_motors().size(), 0.0);
        std::vector<double> arm_joint_velocities(openarm->get_arm().get_motors().size(), 0.0);

        std::vector<double> gripper_joint_positions(openarm->get_gripper().get_motors().size(),
                                                    0.0);
        std::vector<double> gripper_joint_velocities(openarm->get_gripper().get_motors().size(),
                                                     0.0);

        std::vector<double> grav_torques(openarm->get_arm().get_motors().size(), 0.0);

        while (keep_running) {
            // Send command and receive response to get fresh motor state
            openarm->refresh_all();
            openarm->recv_all();

            frame_count++;
            auto current_time = std::chrono::high_resolution_clock::now();

            // Calculate and display Hz every second
            auto time_since_last_display = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               current_time - last_hz_display)
                                               .count();
            auto motors = openarm->get_arm().get_motors();
            for (size_t i = 0; i < motors.size(); ++i) {
                arm_joint_positions[i] = motors[i].get_position();
                arm_joint_velocities[i] = motors[i].get_velocity();
            }

            arm_dynamics.GetGravity(arm_joint_positions.data(), grav_torques.data());

            if (time_since_last_display >= 1000) {  // Every 1000ms (1 second)
                auto total_time =
                    std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time)
                        .count();
                double hz = (frame_count * 1000.0) / total_time;
                std::cout << "=== Loop Frequency: " << hz << " Hz ===" << std::endl;
                for (size_t i = 0; i < openarm->get_arm().get_motors().size(); ++i) {
                    std::cout << "joint[" << i << "] pos=" << arm_joint_positions[i]
                              << " torque=" << grav_torques[i] << std::endl;
                }
                last_hz_display = current_time;
            }

            std::vector<openarm::damiao_motor::MITParam> cmds;
            cmds.reserve(grav_torques.size());

            std::transform(grav_torques.begin(), grav_torques.end(), std::back_inserter(cmds),
                           [](double t) { return openarm::damiao_motor::MITParam{0, 0, 0, 0, t}; });

            openarm->get_arm().mit_control_all(cmds);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm->disable_all();
        openarm->recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
