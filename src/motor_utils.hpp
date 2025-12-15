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

#pragma once

#include <chrono>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

inline void set_control_mode_all(openarm::can::socket::OpenArm* openarm, uint8_t mode) {
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
