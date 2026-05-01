// Tiny wrapper around sd_notify(3)'s wire protocol so the daemon doesn't
// need to link libsystemd. Talks the AF_UNIX/SOCK_DGRAM datagram protocol
// described in `man sd_notify` against $NOTIFY_SOCKET.
//
// Callers:
//   - main.cpp / http_server.cpp send "READY=1" once the listening socket
//     is up, satisfying the Type=notify start-up handshake.
//   - safety_supervisor.cpp sends "WATCHDOG=1" on its 100 ms tick to keep
//     systemd's WatchdogSec timer fed.

#pragma once

#include <cstring>
#include <string>

#ifdef __linux__
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace edge {

// Send a single sd_notify message. Cheap; opens, sends, closes.
// Silent no-op when $NOTIFY_SOCKET is unset (i.e. running outside systemd).
inline void SystemdNotify(const std::string& message) {
#ifdef __linux__
  const char* socket_path = std::getenv("NOTIFY_SOCKET");
  if (socket_path == nullptr || *socket_path == '\0') return;

  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (socket_path[0] == '@') {
    // Linux abstract socket namespace: leading NUL, rest is the name.
    addr.sun_path[0] = '\0';
    std::strncpy(addr.sun_path + 1, socket_path + 1, sizeof(addr.sun_path) - 2);
  } else {
    std::strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);
  }

  ::sendto(fd, message.data(), message.size(), MSG_NOSIGNAL,
           reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  ::close(fd);
#else
  (void)message;
#endif
}

}  // namespace edge
