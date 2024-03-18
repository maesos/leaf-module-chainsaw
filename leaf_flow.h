/*
 * leaf_utils.h
 *
 *  Created on: Dec. 24, 2021
 *      Author: Someone
 */

#ifndef LEAF_FLOW_H_
#define LEAF_FLOW_H_


// #include <string.h>
// #include <stdio.h>
#include <stdint.h>


// Module ID List
#define NODE_ADDR_ROOT 0x00
#define NODE_ADDR_ATMOSPHERIC 0x01
#define NODE_ADDR_ATMOSPHERIC_PLUS 0x02
#define NODE_ADDR_LWS_ANALOG 0x03
#define NODE_ADDR_SOIL 0x04
#define NODE_ADDR_MOD_ALL 0x7F


// Leaf State Flags
#define LEAF_FLOW_POWERUP       1 << 0
#define LEAF_FLOW_PRE_POKE      1 << 1
#define LEAF_FLOW_POST_POKE     1 << 2
#define LEAF_FLOW_PRE_POWERDOWN 1 << 3
#define LEAF_FLOW_POWERDOWN     1 << 4


// Command Definitions
#define LEAF_FLOW_PKT_SEND_NO_MODNEXT     0x01
#define LEAF_FLOW_PKT_SEND_MODNEXT     0x02

#define LEAF_FLOW_PRE_POKE_TIME_CMD     0x21
#define LEAF_FLOW_PRE_POKE_TIME         0x22
#define LEAF_FLOW_PRE_POWERDOWN_TIME_CMD 0x23
#define LEAF_FLOW_PRE_POWERDOWN_TIME    0x24
#define LEAF_FLOW_POWERDOWN_VETO_RQ_CMD 0x25
#define LEAF_FLOW_POWERDOWN_VETO 0x26

// Timing Minimums 



// Module Definitions 
#define LEAF_MOD_LWS_A_STAT0  1 << 0
#define LEAF_MOD_LWS_A_STAT1  1 << 1
#define LEAF_MOD_LWS_A_STAT2  1 << 2
#define LEAF_MOD_LWS_A_STAT3  1 << 3
#define LEAF_MOD_LWS_A_STAT4  1 << 4
#define LEAF_MOD_LWS_A_STAT5  1 << 5
#define LEAF_MOD_LWS_A_STAT6  1 << 6
#define LEAF_MOD_LWS_A_STAT7  1 << 7


#endif /* LEAF_FLOW_H_ */
