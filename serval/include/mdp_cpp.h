/*
 Copyright (C) 2012-2013 Serval Project Inc.
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdint.h>
/* servald constants.h */
#include <constants.h>

#ifdef __cplusplus 
extern "C" {
#endif

/* things needed for source/error location */
struct __sourceloc {
    const char *file;
    unsigned int line;
    const char *function;
};
extern const struct __sourceloc __whence;
#define __HERE__            ((struct __sourceloc){ .file = __FILE__, .line = __LINE__, .function = __FUNCTION__ })
#define __WHENCE__          (__whence.file ? __whence : __HERE__)

/* sid related definitions */
#define SID_SIZE 32 // == crypto_sign_edwards25519sha512batch_PUBLICKEYBYTES
#define SAS_SIZE 32 // == crypto_sign_edwards25519sha512batch_PUBLICKEYBYTES
typedef struct sid_binary {
    unsigned char binary[SID_SIZE];
} sid_t;

#define SID_ANY         ((sid_t){{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}})
#define SID_BROADCAST   ((sid_t){{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}})
#define BIND_PRIMARY SID_ANY
#define BIND_ALL SID_BROADCAST

typedef uint32_t mdp_port_t;

struct mdp_sockaddr {
  sid_t sid;
  mdp_port_t port;
};

/* mdp header definitions */
#define MDP_FLAG_NO_CRYPT (1<<0)
#define MDP_FLAG_NO_SIGN (1<<1)
#define MDP_FLAG_BIND (1<<2)
#define MDP_FLAG_CLOSE (1<<3)
#define MDP_FLAG_ERROR (1<<4)
#define MDP_FLAG_REUSE (1<<5)

struct mdp_header {
  struct mdp_sockaddr local;
  struct mdp_sockaddr remote;
  uint8_t flags;
  uint8_t qos;
  uint8_t ttl;
};

typedef int64_t time_ms_t;
/* low level V2 mdp interface */
int _mdp_socket(struct __sourceloc);
int _mdp_close(struct __sourceloc, int socket);
int _mdp_send(struct __sourceloc, int socket, const struct mdp_header *header, const uint8_t *payload, size_t len);
ssize_t _mdp_recv(struct __sourceloc, int socket, struct mdp_header *header, uint8_t *payload, size_t max_len);
int _mdp_poll(struct __sourceloc, int socket, time_ms_t timeout_ms);
ssize_t mdp_poll_recv(int mdp_sock, time_ms_t deadline, struct mdp_header *rev_header, unsigned char *payload, size_t buffer_size);
int _mdp_bind(struct __sourceloc __whence, int socket, struct mdp_sockaddr *local_addr);

#define mdp_socket()      _mdp_socket(__WHENCE__)
#define mdp_close(s)      _mdp_close(__WHENCE__, (s))
#define mdp_send(s,h,p,l) _mdp_send(__WHENCE__, (s), (h), (p), (l))
#define mdp_recv(s,h,p,l) _mdp_recv(__WHENCE__, (s), (h), (p), (l))
#define mdp_poll(s,t)     _mdp_poll(__WHENCE__, (s), (t))
#define mdp_bind(s,a)     _mdp_bind(__WHENCE__, (s), (a))

#ifdef __cplusplus
}
#endif