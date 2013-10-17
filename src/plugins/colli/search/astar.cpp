//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$               */
/*                                                                      */
/* Description: This is the AStar-implementation for A* of Colli-A*     */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is a high efficient implementation. Therefore this code   */
/*       does not always look very nice here. So be patient and try to  */
/*       understand what I was trying to implement here.                */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#include "astar.h"
#include "../utils/occupancygrid/occupancygrid.h"
#include "../common/defines.h"

#include <utils/math/types.h>
#include <logging/logger.h>
#include <config/config.h>

using namespace std;

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Constructor.
 *  This constructor does several things ;-)
 *  It gets an occupancy grid for the local pointer to garant fast access,
 *   and queries the settings for the grid.
 *  After that several states are initialized. This is done for speed purposes
 *   again, cause only here new is called in this code..
 *  Afterwards the Openlist, closedlist and states for A* are initialized.
 */
CAStar::CAStar( OccupancyGrid * occGrid, Logger* logger, Configuration* config )
 : logger_( logger )
{
  logger_->log_info("AStar", "(Constructor): Initializing AStar");

  m_MaxStates = config->get_int( "/plugins/colli/AStar/MAX_STATES" );

  m_pOccGrid = occGrid;
  m_Width = m_pOccGrid->getWidth() - 1;
  m_Height = m_pOccGrid->getHeight() - 1;

  m_AStarStateCount = 0;
  m_vAStarStates.reserve( m_MaxStates );

  for( int i = 0; i < m_MaxStates; i++) {
    CAStarState * state = new CAStarState();
    m_vAStarStates[i] = state;
  }

  while ( m_pOpenList.size() > 0 )
    m_pOpenList.pop();

  m_hClosedList.clear();

  logger_->log_info("AStar", "(Constructor): Initializing AStar done");
}



/** Destructor.
 *  This destructor deletes all the states allocated during construction.
 */
CAStar::~CAStar()
{
  logger_->log_info("AStar", "(Destructor): Destroying AStar");
  for( int i = 0; i < m_MaxStates; i++ )
    delete m_vAStarStates[i];
  logger_->log_info("AStar", "(Destructor): Destroying AStar done");
}



/** Solve.
 *  Solve is the externally called method to solve the
 *    assignment by A*. It generates an initial state,
 *    and sets its values accordingly.
 *    This state is put on the openlist, and then
 *    the search is called, after which the solution
 *    sequence is generated by the pointers in all states.
 */
void
CAStar::Solve( const point_t &RoboPos, const point_t &TargetPos, vector<point_t> &solution )
{
  // initialize counter, vectors/lists/queues
  m_AStarStateCount = 0;
  while ( m_pOpenList.size() > 0 )
    m_pOpenList.pop();
  m_hClosedList.clear();
  solution.clear();

  // setting start coordinates
  m_pRoboPos.m_X  = RoboPos.x;
  m_pRoboPos.m_Y  = RoboPos.y;
  m_pTargetState.m_X = TargetPos.x;
  m_pTargetState.m_Y = TargetPos.y;

  // generating initialstate
  CAStarState * initialState = m_vAStarStates[++m_AStarStateCount];
  initialState->m_X = m_pRoboPos.m_X;
  initialState->m_Y = m_pRoboPos.m_Y;
  initialState->m_pFather   = 0;
  initialState->m_PastCost  = 0;
  initialState->m_TotalCost = Heuristic( initialState );

  // performing search
  m_pOpenList.push( initialState );
  GetSolutionSequence( Search(), solution );
}



/* =========================================== */
/* *************** PRIVATE PART ************** */
/* =========================================== */


/** Search.
 *  This is the magic A* algorithm.
 *  Its really easy, you can find it like this everywhere.
 */
CAStarState*
CAStar::Search( )
{
  register CAStarState * best = 0;

  // while the openlist not is empty
  while ( m_pOpenList.size() > 0 ) {
    // get best state
    if ( m_pOpenList.size() > 0 ) {
      best = m_pOpenList.top();
      m_pOpenList.pop( );
    } else
      return 0;

    // check if its a goal.
    if ( IsGoal( best ) )
      return best;
    else if ( m_AStarStateCount > m_MaxStates - 6 ) {
      logger_->log_warn("AStar", "**** Warning: Out of states! Increasing A* MaxStates!");

      for( int i = 0; i < m_MaxStates; i++ )
        delete m_vAStarStates[i];

      m_MaxStates += (int)(m_MaxStates/3.0);

      m_vAStarStates.clear();
      m_vAStarStates.resize( m_MaxStates );
      for( int i = 0; i < m_MaxStates; i++) {
        best = new CAStarState();
        m_vAStarStates[i] = best;
      }
      logger_->log_warn("AStar", "**** Increasing done!");
      return 0;
    }

    // generate all its children
    GenerateChildren( best );
  }

  return 0;
}


/** CalculateKey.
 *  This method produces one unique key for a state for
 *    putting this on the closed list.
 *    It has to be really fast, so the function is not so readable.
 *    What it does is the following: x * 2^14 + y. This is unique,
 *    because first it does a bit shift for 14 bits, and adds (or)
 *    afterwards a number that is smaller tham 14 bits!
 */
int
CAStar::CalculateKey( int x, int y )
{
  return (x << 15) | y;  // This line is a crime! But fast ;-)
}


/** GenerateChildren.
 *  This method generates all children for a given state.
 *   This is done with a little range checking and rule checking.
 *   Afterwards these children are put on the openlist.
 */
void
CAStar::GenerateChildren( CAStarState * father )
{
  register CAStarState * child;
  register int key;

  register float prob;

  if ( father->m_Y > 0 ) {
    prob = m_pOccGrid->getProb( father->m_X, father->m_Y-1 );
    if ( prob != _COLLI_CELL_OCCUPIED_ ) {
      child = m_vAStarStates[++m_AStarStateCount];
      child->m_X = father->m_X;
      child->m_Y = father->m_Y-1;
      key = CalculateKey( child->m_X, child->m_Y );
      if ( m_hClosedList.find( key ) == m_hClosedList.end() ) {
        child->m_pFather = father;
        child->m_PastCost = father->m_PastCost + (int)prob;
        child->m_TotalCost = child->m_PastCost + Heuristic( child );
        m_pOpenList.push( child );
        m_hClosedList[key] = key;

      } else
        --m_AStarStateCount;
    }
  }

  if ( father->m_Y < (signed int)m_Height ) {
    prob = m_pOccGrid->getProb( father->m_X, father->m_Y+1 );
    if ( prob != _COLLI_CELL_OCCUPIED_ ) {
      child = m_vAStarStates[++m_AStarStateCount];
      child->m_X = father->m_X;
      child->m_Y = father->m_Y+1;
      key = CalculateKey( child->m_X, child->m_Y );
      if ( m_hClosedList.find( key ) == m_hClosedList.end() ) {
        child->m_pFather = father;
        child->m_PastCost = father->m_PastCost + (int)prob;
        child->m_TotalCost = child->m_PastCost + Heuristic( child );
        m_pOpenList.push( child );
        m_hClosedList[key] = key;

      } else
      --m_AStarStateCount;
    }
  }

  if ( father->m_X > 0 ) {
    prob = m_pOccGrid->getProb( father->m_X-1, father->m_Y );
    if ( prob != _COLLI_CELL_OCCUPIED_ ) {
      child = m_vAStarStates[++m_AStarStateCount];
      child->m_X = father->m_X-1;
      child->m_Y = father->m_Y;
      key = CalculateKey( child->m_X, child->m_Y );
      if ( m_hClosedList.find( key ) == m_hClosedList.end() ) {
        child->m_pFather = father;
        child->m_PastCost = father->m_PastCost + (int)prob;
        child->m_TotalCost = child->m_PastCost + Heuristic( child );
        m_pOpenList.push( child );
        m_hClosedList[key] = key;

      } else
      --m_AStarStateCount;
    }
  }

  if ( father->m_X < (signed int)m_Width ) {
    prob = m_pOccGrid->getProb( father->m_X+1, father->m_Y );
    if ( prob != _COLLI_CELL_OCCUPIED_ ) {
      child = m_vAStarStates[++m_AStarStateCount];
      child->m_X = father->m_X+1;
      child->m_Y = father->m_Y;
      key = CalculateKey( child->m_X, child->m_Y );
      if ( m_hClosedList.find( key ) == m_hClosedList.end() ) {
        child->m_pFather = father;
        child->m_PastCost = father->m_PastCost + (int)prob;
        child->m_TotalCost = child->m_PastCost + Heuristic( child );
        m_pOpenList.push( child );
        m_hClosedList[key] = key;

      } else
      --m_AStarStateCount;
    }
  }

}


/** Heuristic.
 *  This method calculates the heuristic value for a given
 *    state. This is done by the manhatten distance here,
 *    because we are calculating on a grid...
 */
int
CAStar::Heuristic( CAStarState * state )
{
  //  return (int)( abs( state->m_X - m_pTargetState.m_X ));
  return (int)( abs( state->m_X - m_pTargetState.m_X ) +
                abs( state->m_Y - m_pTargetState.m_Y ) );
}


/** IsGoal.
 *  This method checks, if a state is a goal state.
 */
bool
CAStar::IsGoal( CAStarState * state )
{
  return ( (m_pTargetState.m_X == state->m_X) &&
           (m_pTargetState.m_Y == state->m_Y) );
}


/** GetSolutionSequence.
 *  This one enqueues the way of a node back to its root through the
 *    tree into the solution/plan vector.
 */
void
CAStar::GetSolutionSequence( CAStarState * node, vector<point_t> &solution )
{
  register CAStarState * state = node;
  while ( state != 0 ) {
    solution.insert( solution.begin(), point_t( state->m_X, state->m_Y ) );
    state = state->m_pFather;
  }
  //logger_->log_debug("AStar", "(GetSolutionSequence): Solutionsize=%u  , Used states=%i",
  //                   solution.size(), m_AStarStateCount);
}


/* =========================================================================== */
/* =========================================================================== */
/*        ** ** ** ** ** ASTAR STUFF END HERE ** ** ** ** **                   */
/* =========================================================================== */
/* =========================================================================== */

point_t
CAStar::RemoveTargetFromObstacle( int targetX, int targetY, int stepX, int stepY  )
{
  // initializing lists...
  while ( m_pOpenList.size() > 0 )
    m_pOpenList.pop();

  m_hClosedList.clear();
  m_AStarStateCount = 0;
  // starting fill algorithm by putting first state in openlist
  CAStarState * initialState = m_vAStarStates[++m_AStarStateCount];
  initialState->m_X = targetX;
  initialState->m_Y = targetY;
  initialState->m_TotalCost = 0;
  m_pOpenList.push( initialState );
  // search algorithm by gridfilling
  register CAStarState * child;
  register CAStarState * father;
  register int key;

  while ( !(m_pOpenList.empty()) && (m_AStarStateCount < m_MaxStates - 6) ) {
    father = m_pOpenList.top();
    m_pOpenList.pop();
    key = CalculateKey( father->m_X, father->m_Y );

    if ( m_hClosedList.find( key ) == m_hClosedList.end() ) {
      m_hClosedList[key] = key;
      // generiere zwei kinder. wenn besetzt, pack sie an das ende
      //   der openlist mit kosten + 1, sonst return den Knoten
      if ( (father->m_X > 1) && ( father->m_X < (signed)m_Width-2 ) ) {
        child = m_vAStarStates[++m_AStarStateCount];
        child->m_X = father->m_X + stepX;
        child->m_Y = father->m_Y;
        child->m_TotalCost = father->m_TotalCost+1;
        key = CalculateKey( child->m_X, child->m_Y );
        if ( m_pOccGrid->getProb( child->m_X, child->m_Y ) == _COLLI_CELL_NEAR_ )
          return point_t( child->m_X, child->m_Y );
        else if ( m_hClosedList.find( key ) == m_hClosedList.end() )
          m_pOpenList.push( child );
      }

      if ( (father->m_Y > 1) && (father->m_Y < (signed)m_Height-2) ) {
        child = m_vAStarStates[++m_AStarStateCount];
        child->m_X = father->m_X;
        child->m_Y = father->m_Y + stepY;
        child->m_TotalCost = father->m_TotalCost+1;
        key = CalculateKey( child->m_X, child->m_Y );
        if ( m_pOccGrid->getProb( child->m_X, child->m_Y ) == _COLLI_CELL_NEAR_ )
          return point_t( child->m_X, child->m_Y );
        else if ( m_hClosedList.find( key ) == m_hClosedList.end() )
          m_pOpenList.push( child );
      }
    }

  }

  logger_->log_debug("AStar", "Failed to get a modified targetpoint");
  return point_t( targetX, targetY );
}

} // namespace fawkes