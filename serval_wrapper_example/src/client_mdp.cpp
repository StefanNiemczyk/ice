/*
 * client_mdp.cpp
 *
 *  Created on: Aug 31, 2016
 *      Author: sni
 */

#include <iostream>
#include <cstring>
#include <signal.h>

#include <serval_interface.h>

#define PORT 8042

using namespace ice;

int
main(int argc, char *argv[])
{
        int sock;
        struct sigaction act;

        /* read sid from command line parameter */
        if (argc < 3) {
                std::cerr << "usage: " << argv[0] << " SERVER_SID" << std::endl;
                return -1;
        }

        if ((sock = mdp_socket()) < 0) {
                std::cerr << "error creating mdp socket" << std::endl;
                return -1;
        }

        struct mdp_sockaddr sockaddr;
        sockaddr.port = PORT;
        serval_interface::sidToArray(argv[2], sockaddr.sid.binary);


       // MDPSocket client(sock, PORT, serval_interface::arrayToSid(sockaddr.sid.binary));

        /* main loop */
        std::cout << "client: starting client..." << std::endl;
        int len = 0;
        uint8_t* payload;

        while (true) {
            if (len % 50 == 0)
            {
              mdp_close(sock);
              if ((sock = mdp_socket()) < 0) {
                     std::cerr << "error creating mdp socket" << std::endl;
                     return -1;
             }
              std::cout << "created new socket" << std::endl;
            }

            len++;
            uint8_t msg[700];

//            client.send(argv[1], msg, text.length());

            struct mdp_header header;
            bzero(&header, sizeof(header));

//            serval_interface::sidToArray(this->senderSid, header.local.sid.binary);
            serval_interface::sidToArray(argv[1], header.remote.sid.binary);
            header.remote.port = PORT;
            header.qos = OQ_MESH_MANAGEMENT;
            header.ttl = PAYLOAD_TTL_DEFAULT;
            header.flags |= MDP_FLAG_BIND | MDP_FLAG_NO_CRYPT;

            mdp_send(sock, &header, msg, 700);
        }

        std::cout << "client: cleanup..." << std::endl;

        mdp_close(sock);

        return 0;
}
