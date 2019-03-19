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
#include "acoptitrack.h"

#include <err.h>
#include <errno.h>
#include <stdio.h>

#include "gazebo/gazebo_config.h"
#if GAZEBO_MAJOR_VERSION < 8
# include "gazebo/math/gzmath.hh"
#endif

#include "natnet.h"

#define NAT_PING			0
#define NAT_PINGRESPONSE		1
#define NAT_REQUEST			2
#define NAT_RESPONSE			3
#define NAT_REQUEST_MODELDEF		4
#define NAT_MODELDEF			5
#define NAT_REQUEST_FRAMEOFDATA		6
#define NAT_FRAMEOFDATA			7
#define NAT_MESSAGESTRING		8
#define NAT_UNRECOGNIZED_REQUEST	100
#define NAT_UNDEFINED			999999.9999

#define NAT_DESCR_MSET			0
#define NAT_DESCR_BODY			1
#define NAT_DESCR_SKEL			2


/* --- natnet_process_msg -------------------------------------------------- */

int
natnet_process_msg(struct natnet_packet *msg, ssize_t len,
                   const std::set<link_info> &links)
{
  using namespace gazebo;

  union { char *c; uint32_t *u; float *f; } p;

  if (len < 4) return EINVAL;


  switch (msg->id) {
    case NAT_PING:
      if (msg->bytes != 0) return EINVAL;

      snprintf(msg->data.sender.name, MAX_NAMELENGTH, "gazebo");
      msg->data.sender.version[0] = GAZEBO_MAJOR_VERSION;
      msg->data.sender.version[1] = GAZEBO_MINOR_VERSION;
      msg->data.sender.version[2] = GAZEBO_PATCH_VERSION;
      msg->data.sender.version[3] = 0;
      msg->data.sender.natnet_version[0] = 2;
      msg->data.sender.natnet_version[1] = 9;
      msg->data.sender.natnet_version[2] = 0;
      msg->data.sender.natnet_version[3] = 0;

      msg->id = NAT_PINGRESPONSE;
      msg->bytes = sizeof(msg->data.sender);
      break;

    case NAT_REQUEST_MODELDEF: {
      std::set<link_info>::iterator i;
      int n;

      if (msg->bytes != 0) return EINVAL;

      p.c = msg->data.c;
      *p.u++ = links.size();
      for (i = links.begin(); i != links.end(); ++i) {
        *p.u++ = NAT_DESCR_BODY;

        n = snprintf(p.c, MAX_NAMELENGTH, "%s", i->name.c_str());
        p.c += n+1;
        *p.u++ = i->link->GetId();
        *p.u++ = i->link->GetParentId();
        *p.f++ = 0.;
        *p.f++ = 0.;
        *p.f++ = 0.;
      }

      msg->id = NAT_MODELDEF;
      msg->bytes = p.c - msg->data.c;
      break;
    }

    case NAT_FRAMEOFDATA:
      if (msg->bytes != 0) return EINVAL;
      msg->bytes = 0;
      break;

    default: return EINVAL;
  }

  return 0;
}


/* --- natnet_frameofdata -------------------------------------------------- */

void
natnet_frameofdata(uint32_t seq, double ts, double latency, double noise,
                   const std::set<link_info> &links, struct natnet_packet *msg)
{
  using namespace gazebo;

  union { char *c; uint16_t *s; uint32_t *u; float *f; double *d; } p;
  std::set<link_info>::iterator i;
#if GAZEBO_MAJOR_VERSION < 8
  math::Pose lpose, pose;
#else
  ignition::math::Pose3d lpose, pose;
#endif

  p.c = msg->data.c;
  *p.u++ = seq;

  *p.u++ = 0; /* no marker sets */
  *p.u++ = 0; /* no unidentified markers */

  /* rigid bodies */
  *p.u++ = links.size();
  for (i = links.begin(); i != links.end(); ++i) {
    *p.u++ = i->link->GetId();

    if (noise < 1e-4)
      lpose = i->pose;
    else {
#if GAZEBO_MAJOR_VERSION < 8
      lpose = math::Pose(
        math::Rand::GetDblNormal(0., noise),
        math::Rand::GetDblNormal(0., noise),
        math::Rand::GetDblNormal(0., noise),
        math::Rand::GetDblNormal(0., noise),
        math::Rand::GetDblNormal(0., noise),
        math::Rand::GetDblNormal(0., noise));
#else
      lpose = ignition::math::Pose3d(
        ignition::math::Rand::DblNormal(0., noise),
        ignition::math::Rand::DblNormal(0., noise),
        ignition::math::Rand::DblNormal(0., noise),
        ignition::math::Rand::DblNormal(0., noise),
        ignition::math::Rand::DblNormal(0., noise),
        ignition::math::Rand::DblNormal(0., noise));
#endif
      lpose = lpose + i->pose;
    }
    pose = i->offset + lpose;

#if GAZEBO_MAJOR_VERSION < 8
    *p.f++ = pose.pos.y;	/* Y up, X left, Z front */
    *p.f++ = pose.pos.z;
    *p.f++ = pose.pos.x;
    *p.f++ = pose.rot.y;
    *p.f++ = pose.rot.z;
    *p.f++ = pose.rot.x;
    *p.f++ = pose.rot.w;
#else
    *p.f++ = pose.Pos().Y();	/* Y up, X left, Z front */
    *p.f++ = pose.Pos().Z();
    *p.f++ = pose.Pos().X();
    *p.f++ = pose.Rot().Y();
    *p.f++ = pose.Rot().Z();
    *p.f++ = pose.Rot().X();
    *p.f++ = pose.Rot().W();
#endif

    *p.u++ = 1; /* pretend there is one marker, in case one need this */
#if GAZEBO_MAJOR_VERSION < 8
    *p.f++ = lpose.pos.y;
    *p.f++ = lpose.pos.z;
    *p.f++ = lpose.pos.x;
#else
    *p.f++ = lpose.Pos().Y();
    *p.f++ = lpose.Pos().Z();
    *p.f++ = lpose.Pos().X();
#endif
    *p.u++ = i->link->GetId();
    *p.f++ = 1e-3;	/* marker size */

    *p.f++ = noise < 1e-4 ? 1e-4 : noise;	/* mean error */
    *p.s++ = 1;		/* flags (1 means tracked) */
  }

  *p.u++ = 0;		/* no skeletons */
  *p.u++ = 0;		/* no labeled markers */
  *p.u++ = 0;		/* no force plate */

  *p.f++ = latency;
  *p.u++ = 0;		/* timecode (frame) */
  *p.u++ = 0;		/* timecode (subframe) */
  *p.d++ = ts;		/* timestamp */

  *p.s++ = 0;		/* flags (2 means refresh body list) */
  *p.u++ = 0;		/* unused */

  msg->id = NAT_FRAMEOFDATA;
  msg->bytes = p.c - msg->data.c;
}
