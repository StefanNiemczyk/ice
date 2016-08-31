/*
 * server_mdp.cpp
 *
 *  Created on: Aug 31, 2016
 *      Author: sni
 */

#include <iostream>
#include <cstring>
#include <signal.h>
#include <memory>

#include <serval_interface.h>
#include <serval_wrapper/MDPSocket.h>

#define PORT 8042

static int quit = 0;

void
cleanup(int sig)
{
        quit  = 1;
}

using namespace ice;

int main(int argc, char *argv[])
{
  /* read sid from command line parameter */
  if (argc < 2) {
          std::cerr << "usage: " << argv[0] << " SERVER_SID" << std::endl;
          return -1;
  }

        int sock;
        struct sigaction act;
        struct mdp_sockaddr remote_addr;

        if ((sock = mdp_socket()) < 0)
        {
          std::cerr << "Error creating socket" << std::endl;
          return 0;
        }

        struct mdp_sockaddr sockaddr;
        sockaddr.port = PORT;
        serval_interface::sidToArray(argv[1], sockaddr.sid.binary);
        if (mdp_bind(sock, &sockaddr) != 0)
        {
          std::cerr << "Error binding socket to port" << std::endl;
          return 0;
        }

        //auto server = std::make_shared<MDPSocket>(sock, PORT, argv[1]);
        std::string senderSid;
        uint8_t buffer[1024];

        /* cleanup on CTRL-C */
        memset(&act, 0, sizeof(act));
        act.sa_handler = &cleanup;
        if (sigaction(SIGINT, &act, NULL) < 0) {
                fprintf(stderr, "error registering signal handler\n");
                return -1;
        }

        while(quit == 0)
        {
          std::cout << "-------------------------------------------" << std::endl;
//          int size = server->receive(senderSid, buffer, 1024);
          struct mdp_header header;
          int size = mdp_recv(sock, &header, buffer, 1024);

          if (size == 0)
            continue;

          std::cout << buffer << std::endl;
        }

        std::cout << "client: cleanup..." << std::endl;

        mdp_close(sock);

        return 0;
}




