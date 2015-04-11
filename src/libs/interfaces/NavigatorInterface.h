
/***************************************************************************
 *  NavigatorInterface.h - Fawkes BlackBoard Interface - NavigatorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2009  Martin Liebenberg, Daniel Beck, Tim Niemueller
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __INTERFACES_NAVIGATORINTERFACE_H_
#define __INTERFACES_NAVIGATORINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class NavigatorInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(NavigatorInterface)
 /// @endcond
 public:
  /* constants */
  static const uint32_t ERROR_NONE;
  static const uint32_t ERROR_MOTOR;
  static const uint32_t ERROR_OBSTRUCTION;
  static const uint32_t ERROR_UNKNOWN_PLACE;
  static const uint32_t FLAG_NONE;
  static const uint32_t FLAG_CART_GOTO;
  static const uint32_t FLAG_POLAR_GOTO;
  static const uint32_t FLAG_PLACE_GOTO;
  static const uint32_t FLAG_UPDATES_DEST_DIST;
  static const uint32_t FLAG_SECURITY_DISTANCE;
  static const uint32_t FLAG_ESCAPING;

  /** Drive modes enum */
  typedef enum {
    MovingNotAllowed /**< Moving not allowed constant */,
    Forward /**< Moving forward constant */,
    AllowBackward /**< Moving allow backward constant */,
    Backward /**< Moving backward constant */,
    ESCAPE /**< Escape constant */
  } DriveMode;
  const char * tostring_DriveMode(DriveMode value) const;

  /** Orientation mode enum */
  typedef enum {
    OrientAtTarget /**< Orient when at target, if orientation is given */,
    OrientDuringTravel /**< Orient during travel BUT NOT at target, if omnidirectional platform and orientation is given */
  } OrientationMode;
  const char * tostring_OrientationMode(OrientationMode value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t flags; /**< Bit-wise combination of
    FLAG_* constants denoting navigator component features. */
    float x; /**< Current X-coordinate in the navigator coordinate system. */
    float y; /**< Current Y-coordinate in the navigator coordinate system. */
    float dest_x; /**< X-coordinate of the current destination, or 0.0 if no target has been set. */
    float dest_y; /**< Y-coordinate of the current destination, or 0.0 if no target has been set. */
    float dest_ori; /**< Orientation of the current destination, or 0.0 if no target has been set. */
    float dest_dist; /**< Distance to destination in m. */
    uint32_t msgid; /**< The ID of the message that is currently being
      processed, or 0 if no message is being processed. */
    bool final; /**< True, if the last goto command has been finished,
      false if it is still running */
    uint32_t error_code; /**< Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination). */
    float max_velocity; /**< Maximum velocity */
    float max_rotation; /**< Maximum rotation velocity */
    float security_distance; /**< Security distance to keep to obstacles */
    bool escaping_enabled; /**< This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape. */
    int32_t drive_mode; /**< Current drive mode */
    bool auto_drive_mode; /**< True, if the drive mode should be automatically decided each time.
      False, if the drive mode should not automatically change, which is the case when sending
      a SetAutoDriveMode-message (otherwise the navigator might ignore that value). */
    bool stop_at_target; /**< Stop when target is reached? */
    int32_t orientation_mode; /**< Mode how/when to orientate if orientation is given */
  } NavigatorInterface_data_t;
#pragma pack(pop)

  NavigatorInterface_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
 public:
  /* messages */
  class StopMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } StopMessage_data_t;
#pragma pack(pop)

    StopMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    StopMessage();
    ~StopMessage();

    StopMessage(const StopMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class TurnMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float angle; /**< Angle of the turn. */
      float velocity; /**< The desired turning velocity in rad/s,
      set to zero to use default value. */
    } TurnMessage_data_t;
#pragma pack(pop)

    TurnMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    TurnMessage(const float ini_angle, const float ini_velocity);
    TurnMessage();
    ~TurnMessage();

    TurnMessage(const TurnMessage *m);
    /* Methods */
    float angle() const;
    void set_angle(const float new_angle);
    size_t maxlenof_angle() const;
    float velocity() const;
    void set_velocity(const float new_velocity);
    size_t maxlenof_velocity() const;
    virtual Message * clone() const;
  };

  class CartesianGotoMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X-coordinate of the target, in the robot's coordinate system. */
      float y; /**< Y-coordinate of the target, in the robot's coordinate system. */
      float orientation; /**< The desired orientation of the robot at the target. */
    } CartesianGotoMessage_data_t;
#pragma pack(pop)

    CartesianGotoMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    CartesianGotoMessage(const float ini_x, const float ini_y, const float ini_orientation);
    CartesianGotoMessage();
    ~CartesianGotoMessage();

    CartesianGotoMessage(const CartesianGotoMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float orientation() const;
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
    virtual Message * clone() const;
  };

  class PolarGotoMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float phi; /**< Angle between the robot's front and the target. */
      float dist; /**< Distance to the target. */
      float orientation; /**< The desired orientation of the robot at the target. */
    } PolarGotoMessage_data_t;
#pragma pack(pop)

    PolarGotoMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    PolarGotoMessage(const float ini_phi, const float ini_dist, const float ini_orientation);
    PolarGotoMessage();
    ~PolarGotoMessage();

    PolarGotoMessage(const PolarGotoMessage *m);
    /* Methods */
    float phi() const;
    void set_phi(const float new_phi);
    size_t maxlenof_phi() const;
    float dist() const;
    void set_dist(const float new_dist);
    size_t maxlenof_dist() const;
    float orientation() const;
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
    virtual Message * clone() const;
  };

  class PlaceGotoMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char place[64]; /**< Place to go to. */
    } PlaceGotoMessage_data_t;
#pragma pack(pop)

    PlaceGotoMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    PlaceGotoMessage(const char * ini_place);
    PlaceGotoMessage();
    ~PlaceGotoMessage();

    PlaceGotoMessage(const PlaceGotoMessage *m);
    /* Methods */
    char * place() const;
    void set_place(const char * new_place);
    size_t maxlenof_place() const;
    virtual Message * clone() const;
  };

  class PlaceWithOriGotoMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char place[64]; /**< Place to go to. */
      float orientation; /**< The desired orientation of the robot at the target. */
    } PlaceWithOriGotoMessage_data_t;
#pragma pack(pop)

    PlaceWithOriGotoMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    PlaceWithOriGotoMessage(const char * ini_place, const float ini_orientation);
    PlaceWithOriGotoMessage();
    ~PlaceWithOriGotoMessage();

    PlaceWithOriGotoMessage(const PlaceWithOriGotoMessage *m);
    /* Methods */
    char * place() const;
    void set_place(const char * new_place);
    size_t maxlenof_place() const;
    float orientation() const;
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
    virtual Message * clone() const;
  };

  class ObstacleMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X-coordinate of the obstacle. */
      float y; /**< Y-coordinate of the obstacle. */
      float width; /**< Width of the obstacle. */
    } ObstacleMessage_data_t;
#pragma pack(pop)

    ObstacleMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    ObstacleMessage(const float ini_x, const float ini_y, const float ini_width);
    ObstacleMessage();
    ~ObstacleMessage();

    ObstacleMessage(const ObstacleMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float width() const;
    void set_width(const float new_width);
    size_t maxlenof_width() const;
    virtual Message * clone() const;
  };

  class ResetOdometryMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ResetOdometryMessage_data_t;
#pragma pack(pop)

    ResetOdometryMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    ResetOdometryMessage();
    ~ResetOdometryMessage();

    ResetOdometryMessage(const ResetOdometryMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class SetMaxVelocityMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float max_velocity; /**< Maximum velocity */
    } SetMaxVelocityMessage_data_t;
#pragma pack(pop)

    SetMaxVelocityMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetMaxVelocityMessage(const float ini_max_velocity);
    SetMaxVelocityMessage();
    ~SetMaxVelocityMessage();

    SetMaxVelocityMessage(const SetMaxVelocityMessage *m);
    /* Methods */
    float max_velocity() const;
    void set_max_velocity(const float new_max_velocity);
    size_t maxlenof_max_velocity() const;
    virtual Message * clone() const;
  };

  class SetMaxRotationMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float max_rotation; /**< Maximum rotation velocity */
    } SetMaxRotationMessage_data_t;
#pragma pack(pop)

    SetMaxRotationMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetMaxRotationMessage(const float ini_max_rotation);
    SetMaxRotationMessage();
    ~SetMaxRotationMessage();

    SetMaxRotationMessage(const SetMaxRotationMessage *m);
    /* Methods */
    float max_rotation() const;
    void set_max_rotation(const float new_max_rotation);
    size_t maxlenof_max_rotation() const;
    virtual Message * clone() const;
  };

  class SetEscapingMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      bool escaping_enabled; /**< This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape. */
    } SetEscapingMessage_data_t;
#pragma pack(pop)

    SetEscapingMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetEscapingMessage(const bool ini_escaping_enabled);
    SetEscapingMessage();
    ~SetEscapingMessage();

    SetEscapingMessage(const SetEscapingMessage *m);
    /* Methods */
    bool is_escaping_enabled() const;
    void set_escaping_enabled(const bool new_escaping_enabled);
    size_t maxlenof_escaping_enabled() const;
    virtual Message * clone() const;
  };

  class SetSecurityDistanceMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float security_distance; /**< Security distance to keep to obstacles */
    } SetSecurityDistanceMessage_data_t;
#pragma pack(pop)

    SetSecurityDistanceMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetSecurityDistanceMessage(const float ini_security_distance);
    SetSecurityDistanceMessage();
    ~SetSecurityDistanceMessage();

    SetSecurityDistanceMessage(const SetSecurityDistanceMessage *m);
    /* Methods */
    float security_distance() const;
    void set_security_distance(const float new_security_distance);
    size_t maxlenof_security_distance() const;
    virtual Message * clone() const;
  };

  class SetDriveModeMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t drive_mode; /**< Current drive mode */
    } SetDriveModeMessage_data_t;
#pragma pack(pop)

    SetDriveModeMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetDriveModeMessage(const DriveMode ini_drive_mode);
    SetDriveModeMessage();
    ~SetDriveModeMessage();

    SetDriveModeMessage(const SetDriveModeMessage *m);
    /* Methods */
    DriveMode drive_mode() const;
    void set_drive_mode(const DriveMode new_drive_mode);
    size_t maxlenof_drive_mode() const;
    virtual Message * clone() const;
  };

  class SetStopAtTargetMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      bool stop_at_target; /**< Stop when target is reached? */
    } SetStopAtTargetMessage_data_t;
#pragma pack(pop)

    SetStopAtTargetMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetStopAtTargetMessage(const bool ini_stop_at_target);
    SetStopAtTargetMessage();
    ~SetStopAtTargetMessage();

    SetStopAtTargetMessage(const SetStopAtTargetMessage *m);
    /* Methods */
    bool is_stop_at_target() const;
    void set_stop_at_target(const bool new_stop_at_target);
    size_t maxlenof_stop_at_target() const;
    virtual Message * clone() const;
  };

  class SetOrientationModeMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t orientation_mode; /**< Mode how/when to orientate if orientation is given */
    } SetOrientationModeMessage_data_t;
#pragma pack(pop)

    SetOrientationModeMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    SetOrientationModeMessage(const OrientationMode ini_orientation_mode);
    SetOrientationModeMessage();
    ~SetOrientationModeMessage();

    SetOrientationModeMessage(const SetOrientationModeMessage *m);
    /* Methods */
    OrientationMode orientation_mode() const;
    void set_orientation_mode(const OrientationMode new_orientation_mode);
    size_t maxlenof_orientation_mode() const;
    virtual Message * clone() const;
  };

  class ResetParametersMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ResetParametersMessage_data_t;
#pragma pack(pop)

    ResetParametersMessage_data_t *data;

  interface_enum_map_t enum_map_DriveMode;
  interface_enum_map_t enum_map_OrientationMode;
   public:
    ResetParametersMessage();
    ~ResetParametersMessage();

    ResetParametersMessage(const ResetParametersMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  NavigatorInterface();
  ~NavigatorInterface();

 public:
  /* Methods */
  uint32_t flags() const;
  void set_flags(const uint32_t new_flags);
  size_t maxlenof_flags() const;
  float x() const;
  void set_x(const float new_x);
  size_t maxlenof_x() const;
  float y() const;
  void set_y(const float new_y);
  size_t maxlenof_y() const;
  float dest_x() const;
  void set_dest_x(const float new_dest_x);
  size_t maxlenof_dest_x() const;
  float dest_y() const;
  void set_dest_y(const float new_dest_y);
  size_t maxlenof_dest_y() const;
  float dest_ori() const;
  void set_dest_ori(const float new_dest_ori);
  size_t maxlenof_dest_ori() const;
  float dest_dist() const;
  void set_dest_dist(const float new_dest_dist);
  size_t maxlenof_dest_dist() const;
  uint32_t msgid() const;
  void set_msgid(const uint32_t new_msgid);
  size_t maxlenof_msgid() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  uint32_t error_code() const;
  void set_error_code(const uint32_t new_error_code);
  size_t maxlenof_error_code() const;
  float max_velocity() const;
  void set_max_velocity(const float new_max_velocity);
  size_t maxlenof_max_velocity() const;
  float max_rotation() const;
  void set_max_rotation(const float new_max_rotation);
  size_t maxlenof_max_rotation() const;
  float security_distance() const;
  void set_security_distance(const float new_security_distance);
  size_t maxlenof_security_distance() const;
  bool is_escaping_enabled() const;
  void set_escaping_enabled(const bool new_escaping_enabled);
  size_t maxlenof_escaping_enabled() const;
  DriveMode drive_mode() const;
  void set_drive_mode(const DriveMode new_drive_mode);
  size_t maxlenof_drive_mode() const;
  bool is_auto_drive_mode() const;
  void set_auto_drive_mode(const bool new_auto_drive_mode);
  size_t maxlenof_auto_drive_mode() const;
  bool is_stop_at_target() const;
  void set_stop_at_target(const bool new_stop_at_target);
  size_t maxlenof_stop_at_target() const;
  OrientationMode orientation_mode() const;
  void set_orientation_mode(const OrientationMode new_orientation_mode);
  size_t maxlenof_orientation_mode() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
