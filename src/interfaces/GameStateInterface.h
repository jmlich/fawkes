
/***************************************************************************
 *  GameStateInterface.h - Fawkes BlackBoard Interface - GameStateInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
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

#ifndef __INTERFACES_GAMESTATEINTERFACE_H_
#define __INTERFACES_GAMESTATEINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

class GameStateInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(GameStateInterface)
 /// @endcond
 public:
  /* constants */
  static const unsigned int GS_FROZEN;
  static const unsigned int GS_PLAY;
  static const unsigned int GS_KICK_OFF;
  static const unsigned int GS_DROP_BALL;
  static const unsigned int GS_PENALTY;
  static const unsigned int GS_CORNER_KICK;
  static const unsigned int GS_THROW_IN;
  static const unsigned int GS_FREE_KICK;
  static const unsigned int GS_GOAL_KICK;
  static const unsigned int GS_HALF_TIME;
  static const unsigned int GS_SPL_INITIAL;
  static const unsigned int GS_SPL_READY;
  static const unsigned int GS_SPL_SET;
  static const unsigned int GS_SPL_PLAY;
  static const unsigned int GS_SPL_FINISHED;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_team_t.
       */
  typedef enum {
    TEAM_NONE /**< No team, not team-specific */,
    TEAM_CYAN /**< Cyan team */,
    TEAM_MAGENTA /**< Magenta team */,
    TEAM_BOTH /**< Both teams */
  } if_gamestate_team_t;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_goalcolor_t.
       */
  typedef enum {
    GOAL_BLUE /**< Blue goal */,
    GOAL_YELLOW /**< Yellow goal */
  } if_gamestate_goalcolor_t;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_half_t.
       */
  typedef enum {
    HALF_FIRST /**< First half */,
    HALF_SECOND /**< Second half */
  } if_gamestate_half_t;

  /** 
        Enumeration defining the different robot roles. Keep in sync with
	worldinfo_gamestate_role_t.
       */
  typedef enum {
    ROLE_GOALIE /**< Goalie */,
    ROLE_DEFENDER /**< Defender */,
    ROLE_MID_LEFT /**< Midfield left */,
    ROLE_MID_RIGHT /**< Midfield right */,
    ROLE_ATTACKER /**< Attacker */
  } if_gamestate_role_t;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int game_state; /**< Current game state */
    unsigned int score_cyan; /**< Score of team cyan */
    unsigned int score_magenta; /**< Score of team magenta */
    if_gamestate_team_t state_team; /**< Team referred to by game state */
    if_gamestate_team_t our_team; /**< Our team color */
    if_gamestate_goalcolor_t our_goal_color; /**< Our own goal color */
    if_gamestate_half_t half; /**< Current game half */
    if_gamestate_role_t role; /**< Current role of this robot */
  } GameStateInterface_data_t;

  GameStateInterface_data_t *data;

 public:
  /* messages */
  class SetTeamColorMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      if_gamestate_team_t our_team; /**< Our team color */
    } SetTeamColorMessage_data_t;

    SetTeamColorMessage_data_t *data;

   public:
    SetTeamColorMessage(const if_gamestate_team_t ini_our_team);
    SetTeamColorMessage();
    ~SetTeamColorMessage();

    SetTeamColorMessage(const SetTeamColorMessage *m);
    /* Methods */
    if_gamestate_team_t our_team() const;
    void set_our_team(const if_gamestate_team_t new_our_team);
    size_t maxlenof_our_team() const;
    virtual Message * clone() const;
  };

  class SetStateTeamMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      if_gamestate_team_t state_team; /**< Team referred to by game state */
    } SetStateTeamMessage_data_t;

    SetStateTeamMessage_data_t *data;

   public:
    SetStateTeamMessage(const if_gamestate_team_t ini_state_team);
    SetStateTeamMessage();
    ~SetStateTeamMessage();

    SetStateTeamMessage(const SetStateTeamMessage *m);
    /* Methods */
    if_gamestate_team_t state_team() const;
    void set_state_team(const if_gamestate_team_t new_state_team);
    size_t maxlenof_state_team() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  GameStateInterface();
  ~GameStateInterface();

 public:
  /* Methods */
  unsigned int game_state() const;
  void set_game_state(const unsigned int new_game_state);
  size_t maxlenof_game_state() const;
  if_gamestate_team_t state_team() const;
  void set_state_team(const if_gamestate_team_t new_state_team);
  size_t maxlenof_state_team() const;
  if_gamestate_team_t our_team() const;
  void set_our_team(const if_gamestate_team_t new_our_team);
  size_t maxlenof_our_team() const;
  if_gamestate_goalcolor_t our_goal_color() const;
  void set_our_goal_color(const if_gamestate_goalcolor_t new_our_goal_color);
  size_t maxlenof_our_goal_color() const;
  if_gamestate_half_t half() const;
  void set_half(const if_gamestate_half_t new_half);
  size_t maxlenof_half() const;
  if_gamestate_role_t role() const;
  void set_role(const if_gamestate_role_t new_role);
  size_t maxlenof_role() const;
  unsigned int score_cyan() const;
  void set_score_cyan(const unsigned int new_score_cyan);
  size_t maxlenof_score_cyan() const;
  unsigned int score_magenta() const;
  void set_score_magenta(const unsigned int new_score_magenta);
  size_t maxlenof_score_magenta() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
