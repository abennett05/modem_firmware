/// modem_types.h
/// - - - - - - - - - - - -
/// This file contains all useful
/// types for the firmware of the
/// Modem Puck.

#pragma once
#include <stdint.h>

/// Constants
/// - - - - - - - - - - - -
/// Immutable values to be 
/// referenced in the modem_firmware.ino
/// side of the project.
/// These values are aimed to
/// define CORE properties
/// of the Modem Puck.

// Device Properties
constexpr uint8_t MAX_DEVICE_NAME_LEN = 32;
constexpr uint8_t MAX_USERS = 8;
constexpr uint8_t MAX_KEYFRAMES = 32;
constexpr uint8_t NUM_LIGHTS = 12;
// User Properties
constexpr uint8_t MAX_USER_NAME_LEN = 24;


/// State
/// - - - - - - - - - - - -
/// This enum represents the 
/// current state of the device.
/// The state determines the power
/// mode of the device.
/// Idle = Low Power / Sleep
/// Active = High Power / Normal
enum class State { Idle, Active };

/// User
/// - - - - - - - - - - - -
/// Defines the properties of
/// each connected user.
/// details include but aren't
/// limited to:
/// USER ID - A unique identifier users
/// CONN HANDLE - BLE Connection Status
/// USER NAME - A user's chosen name
/// COLOR CODE - A user's chosen color
/// ANIMATION ID - A user's chosen animation
/// SOUND ID - A user's chosen sound
/// SCREEN TIME - Total screen time for this user (Seconds)
struct User {
  uint8_t ID;
  uint16_t connHandle;
  char name[MAX_USER_NAME_LEN];
  uint32_t color;
  uint8_t animationID;
  uint8_t soundID;
  uint16_t screenTime;
  // Leave-grace bookkeeping. 0 = a live member. Non-zero = the link dropped
  // spontaneously (NOT an app-requested leave) at this millis() timestamp; the
  // user is held in the roster through LEAVE_GRACE_MS so a quick iOS reopen +
  // app rejoin can reclaim the slot (see reclaimPendingLeave) without flickering
  // off everyone else's leaderboard. loop() finalizes the leave if it expires.
  uint32_t pendingLeaveMS;
};

/// Session
/// - - - - - - - - - - - -
/// Defines the active Session
/// properties include:
/// SESSION ID - Unique ID for Session
/// USERS - Collection of USER
/// USER COUNT - Total of all connected users
/// START TIME MS- Time at start of session (in ms)
struct Session {
  uint8_t ID;
  User connectedUsers[MAX_USERS];
  uint8_t userCount;
  uint32_t startTimeMS;
};

/// Frame
/// - - - - - - - - - - - -
/// Defintion for a single
/// frame of an animation.
/// Each frame contains
/// the value for each LED
/// and the brightness
/// of the device
struct Frame {
  uint8_t brightness;
  uint8_t values[NUM_LIGHTS];
};

/// Animation
/// - - - - - - - - - - - - 
/// Definition for animation
/// datatype. Includes properties
/// such as key frames and 
/// animation ID.
struct Animation {
  uint8_t ID;
  Frame frames[MAX_KEYFRAMES];
};

/// Sound
/// - - - - - - - - - - - -
/// Definition for a sound
/// effect that plays
/// during interaction.
/// Properties include:
/// SOUND ID - Unique ID of a sound
/// PATH - Path in local storage to .mp3 file
struct Sound {
  uint8_t ID;
  char path[64];
};

