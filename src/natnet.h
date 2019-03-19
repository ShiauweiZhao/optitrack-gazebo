/*
 * Copyright (c) 2019 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Tue Mar 19 2019
 */
#ifndef H_NATNET
#define H_NATNET

#include <sys/types.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>

#include <set>

#include "gazebo/physics/physics.hh"

struct link_info {
  std::string name;
  gazebo::physics::LinkPtr link;

#if GAZEBO_MAJOR_VERSION < 8
  gazebo::math::Pose offset;
  mutable gazebo::math::Pose pose;
#else
  ignition::math::Pose3d offset;
  mutable ignition::math::Pose3d pose;
#endif

  bool
  operator<(const link_info &other) const
  {
    return name < other.name;
  }
};

#define MAX_NAMELENGTH 256
#define MAX_PACKETSIZE 0x10000	/* actual packet size is dynamic */

struct natnet_sender {
  char name[MAX_NAMELENGTH];		/* sending app's name */
  unsigned char version[4];		/* app's version [X.X.X.X] */
  unsigned char natnet_version[4];	/* NatNet version [X.X.X.X] */
};

struct natnet_packet {
  uint16_t id;		/* message ID (e.g. NAT_FRAMEOFDATA) */
  uint16_t bytes;	/* actual payload size */
  union {
    char	c[MAX_PACKETSIZE];
    struct natnet_sender sender;
  } data;
};

int	natnet_process_msg(struct natnet_packet *msg, ssize_t len,
                const std::set<link_info> &links);
void	natnet_frameofdata(uint32_t seq, double ts, double latency,
                double noise, const std::set<link_info> &links,
                struct natnet_packet *msg);

#endif /* H_NATNET */
