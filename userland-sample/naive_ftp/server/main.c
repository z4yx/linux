#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <limits.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include "service.h"
#include "network.h"
#include "params.h"

int gIsVerbose;
unsigned short gFtpPort = DEFAULT_FTP_PORT;

void waiting_for_client()
{
    init_cmd_sock();
    if(0 != server_sock_bind_listen(1, gFtpPort)){
        printf("Failed to listen on %d\n", gFtpPort);
        exit(1);
    }
    printf("Listening on %d\n", gFtpPort);
    for(;;) {
        struct sockaddr_in client;
        int conn_fd;
        if(0 != server_sock_accept(1, &client, &conn_fd)){
            perror("accept");
            exit(1);
        }

        int ret = fork();
        if(-1 == ret){
            perror("fork");
            exit(1);
        }
        if(ret == 0){
            close_cmd_sock();
            set_cmd_sock(conn_fd);

            service_handle_client(&client);
            break;
        }else{
            close(conn_fd);
        }
    }

}

int main(int argc, char **argv)
{
    int c;
    int digit_optind = 0;

    printf("%s %s\n", "Naive FTP Server", VERSION);

    while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] = {
            {"port",     required_argument, 0,  'p' },
            {"verbose", no_argument,       0,  'v' },
            {0,         0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "p:v",
                        long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
        case 0:
            // printf("option %s", long_options[option_index].name);
            // if (optarg)
            //     printf(" with arg %s", optarg);
            // printf("\n");
            break;
        case 'p':
            gFtpPort = atoi(optarg);
            break;
        case 'v':
            gIsVerbose = 1;
            break;

        default:
            // printf("unsupported argument: %c\n", c);
            exit(1);
        }
    }

    if (optind < argc) {
        printf("unknown arguments: ");
        while (optind < argc)
            printf("%s ", argv[optind++]);
        printf("\n");
        exit(1);
    }

    waiting_for_client();

    exit(EXIT_SUCCESS);
}
