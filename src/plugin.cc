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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <err.h>
#include <pthread.h>
#include <stddef.h>

#include "boost/bind.hpp"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "natnet.h"

using namespace gazebo;

class optitrack_plugin : public WorldPlugin
{
  struct client_info {
    struct sockaddr_in addr;

    bool
    operator<(const client_info &other) const
      {
        return memcmp(&addr, &other.addr, sizeof(addr)) < 0;
      }
  };

  int fd;
  pthread_t cmd, pub;
  pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

  double hz, rate;
  struct timespec period;

  double noise;
  std::set<link_info> links;
  std::set<client_info> clients;

  event::ConnectionPtr end;

 public:
  /* --- Load -------------------------------------------------------------- */

  void
  Load(physics::WorldPtr world, sdf::ElementPtr sdf)
  {
    struct sockaddr_in addr;
    int port;

    /* get list of links to be published */
    for(sdf::ElementPtr publish = sdf->GetElement("publish");
        publish;
        publish = publish->GetNextElement("publish")) {
      if (!publish->HasElement("link")) {
        warnx("optitrack-gazebo ignoring <publish> without <link> element");
        continue;
      }

      link_info info;

      info.name = publish->Get<std::string>("link");
      info.link = boost::dynamic_pointer_cast<physics::Link>(
#if GAZEBO_MAJOR_VERSION < 8
        world->GetEntity(info.name)
#else
        world->EntityByName(info.name)
#endif
        );
      if (!info.link) {
        warnx("optitrack-gazebo ignoring <publish> for unknown link %s",
              info.name.c_str());
        continue;
      }
      if (publish->HasElement("name"))
        info.name = publish->Get<std::string>("name");
      else
        info.name = info.link->GetScopedName();

#if GAZEBO_MAJOR_VERSION < 8
      if (publish->HasElement("offset"))
        info.offset = publish->Get<math::Pose>("offset");
      else
        info.offset = math::Pose::Zero;
#else
      if (publish->HasElement("offset"))
        info.offset = publish->Get<ignition::math::Pose3d>("offset");
      else
        info.offset = ignition::math::Pose3d::Zero;
#endif

      if (links.insert(info).second)
        warnx("optitrack publishing %s", info.name.c_str());
    }


    /* create command socket */
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      warn("optitrack-gazebo could not create socket");
      gzthrow("optitrack-gazebo could not create socket");
    }

    port = sdf->HasElement("port") ? sdf->Get<int>("port") : 1509;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      warn("optitrack-gazebo could not bind socket on port %d", port);
      close(fd);
      gzthrow("optitrack-gazebo could not bind socket");
    }
    warnx("optitrack listening on port %d", port);


    /* create command and publish threads */
#if GAZEBO_MAJOR_VERSION < 8
    rate = world->GetPhysicsEngine()->GetRealTimeUpdateRate();
#else
    rate = world->Physics()->GetRealTimeUpdateRate();
#endif
    hz = sdf->HasElement("hz") ? sdf->Get<double>("hz") : 100.;
    if (hz < 1./86400) {
      warn("optitrack-gazebo: invalid publish rate less than once a day");
      gzthrow("optitrack-gazebo: invalid publish rate less than once a day");
    }
    noise = sdf->HasElement("noise") ? sdf->Get<double>("noise") : 0.;
    if (noise < 1e-4) noise = 0.;

    period.tv_sec = 1/hz;
    period.tv_nsec = 1e9 * (1/hz - period.tv_sec);

    pthread_create(&cmd, NULL, cmd_thread, this);
    pthread_create(&pub, NULL, pub_thread, this);

    /* plug events */
    end = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&optitrack_plugin::update_end, this));
  }


  /* --- update_end -------------------------------------------------------- */

  void
  update_end()
  {
    std::set<link_info>::iterator i;

    /* cache all links pose */
    pthread_mutex_lock(&mutex);
    for (i = links.begin(); i != links.end(); ++i)
#if GAZEBO_MAJOR_VERSION < 8
      i->pose = i->link->GetWorldPose();
#else
      i->pose = i->link->WorldPose();
#endif
    pthread_mutex_unlock(&mutex);
  }


  /* --- ~optitrack_plugin ------------------------------------------------- */

  ~optitrack_plugin()
  {
    pthread_mutex_lock(&mutex);
    close(fd);
    fd = -1;
    pthread_mutex_unlock(&mutex);
    pthread_join(cmd, NULL);
    pthread_join(pub, NULL);
    warnx("optitrack-gazebo disconnecting");
  }


 private:
  /* --- cmd_thread -------------------------------------------------------- */

  static void *
  cmd_thread(void *data)
  {
    optitrack_plugin *plugin = (optitrack_plugin *)data;
    struct natnet_packet *buffer;
    client_info client;
    socklen_t len;
    ssize_t n;
    int s;

    buffer = new natnet_packet;

    pthread_mutex_lock(&plugin->mutex);
    while(plugin->fd >= 0) {

      pthread_mutex_unlock(&plugin->mutex);
      {
        do {
          len = sizeof(client.addr);
          n = recvfrom(plugin->fd, buffer, MAX_PACKETSIZE,
                       0, (struct sockaddr *)&client.addr, &len);
        } while(n < 0 && errno == EINTR);
      }
      pthread_mutex_lock(&plugin->mutex);
      if (n < 0) { warn("optitrack-gazebo: recvfrom"); break; }
      if (plugin->fd < 0) break;
      if (n == 0) continue;
      if (len > sizeof(client.addr)) continue;

      s = natnet_process_msg(buffer, n, plugin->links);
      if (s) continue;

      if (buffer->bytes)
        sendto(plugin->fd,
               buffer, buffer->bytes + offsetof(struct natnet_packet, data),
               0, (struct sockaddr *)&client.addr, len);

      if (plugin->clients.insert(client).second)
        warnx("optitrack streaming to %s:%d",
              inet_ntoa(client.addr.sin_addr), ntohs(client.addr.sin_port));
    }
    pthread_mutex_unlock(&plugin->mutex);

    delete buffer;
    return NULL;
  }


  /* --- pub_thread -------------------------------------------------------- */

  static void *
  pub_thread(void *data)
  {
    optitrack_plugin *plugin = (optitrack_plugin *)data;
    std::set<client_info>::iterator c;
    struct natnet_packet *buffer;
    struct timespec next;
    uint32_t seq = 0;
    ssize_t n;
    int s;

    clock_gettime(CLOCK_REALTIME, &next);
    buffer = new natnet_packet;

    pthread_mutex_lock(&plugin->mutex);
    while(plugin->fd >= 0) {
      pthread_mutex_unlock(&plugin->mutex);
      {
        /* implement periodic wakeup manually, to not rely on posix timers and
         * the portability issues associated to signals vs. threads */
        next.tv_sec += plugin->period.tv_sec;
        next.tv_nsec += plugin->period.tv_nsec;
        if (next.tv_nsec >= 1000000000) {
          next.tv_nsec -= 1000000000;
          next.tv_sec++;
        }

        do {
          s = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
        } while(s && errno == EINTR);
      }
      pthread_mutex_lock(&plugin->mutex);
      if (s) { warn("optitrack-gazebo: clock_nanosleep"); break; }
      if (plugin->fd < 0) break;

      /* send to clients */
      seq++;
      natnet_frameofdata(
        seq, seq/plugin->hz, 1./plugin->rate, plugin->noise, plugin->links,
        buffer);
      for (c = plugin->clients.begin(); c != plugin->clients.end();) {
        n = sendto(plugin->fd,
                   buffer, buffer->bytes + offsetof(natnet_packet, data),
                   0, (struct sockaddr *)&c->addr, sizeof(struct sockaddr_in));
        if (n == -1) {
          warnx("optitrack cancelling streaming to %s:%d",
                inet_ntoa(c->addr.sin_addr), ntohs(c->addr.sin_port));
#if __cplusplus <= 199711L
          plugin->clients.erase(c++);
#else
          c = plugin->clients.erase(c);
#endif
        } else
          ++c;
      }
    }
    pthread_mutex_unlock(&plugin->mutex);

    delete buffer;
    return NULL;
  }

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(optitrack_plugin);
