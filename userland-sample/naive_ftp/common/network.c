#include "network.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/un.h>
#include <netdb.h>
#include <ifaddrs.h>

static int sock_cmd = -1, sock_data = -1, sock_pasv = -1;

void init_cmd_sock()
{
    sock_cmd = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_cmd < 0){
        perror("control socket");
        exit(1);
    }
}

void init_pasv_sock()
{
    sock_pasv = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_pasv < 0){
        perror("pasv socket");
        exit(1);
    }
}

void close_cmd_sock()
{
    close(sock_cmd);
    sock_cmd = -1;
}

void close_pasv_sock()
{
    close(sock_pasv);
    sock_pasv = -1;
}

int server_sock_bind_listen(int is_cmd, short port)
{
    struct sockaddr_in server_addr;
    int r;

    memset((void*)&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    r = bind(is_cmd ? sock_cmd : sock_pasv, (struct sockaddr*)&server_addr, sizeof(struct sockaddr_in));
    if(r != 0) {
        return -1;
    }
    r = listen(is_cmd ? sock_cmd : sock_pasv, 10);
    return r != 0 ? -1 : 0;
}

int server_sock_accept(int is_cmd, struct sockaddr_in* client, int *conn_fd)
{
    int len = sizeof(struct sockaddr_in);
    int r = accept(is_cmd ? sock_cmd : sock_pasv, (struct sockaddr *)client, (socklen_t *)&len);
    if(r < 0)
        return -1;
    *conn_fd = r;
    return 0;
}

void set_cmd_sock(int fd)
{
    sock_cmd = fd;
}

void set_data_sock(int fd)
{
    sock_data = fd;
}

int connect_server(const char* servername, unsigned short port)
{
    struct hostent *host;
    struct sockaddr_in server;
    memset(&server, 0, sizeof(struct sockaddr_in));
    
    host = gethostbyname(servername);
    if(!host){
        herror("gethostbyname");
        return -1;
    }
    memcpy(&server.sin_addr, host->h_addr, host->h_length);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    if(-1 == connect(sock_cmd,(struct sockaddr *)&server, sizeof(server))){
        perror("failed to connect to server");
        return -1;
    }
    return 0;
}

int connect_data_server(const struct in_addr* data_addr, unsigned short data_port)
{
    struct sockaddr_in server;
    memset(&server, 0, sizeof(struct sockaddr_in));
    // printf("%s ip:%u, port:%u\n", __func__, data_addr->s_addr, data_port);
    memcpy(&server.sin_addr, data_addr, sizeof(struct in_addr));
    server.sin_family = AF_INET;
    server.sin_port = htons(data_port);

    if(-1 == connect(sock_data,(struct sockaddr *)&server, sizeof(server))){
        perror("failed to connect to server");
        return -1;
    }
    return 0;
}

int open_data_sock()
{
    sock_data = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_data < 0){
        // perror("data socket");
        return -1;
    }
    return 0;
}

void close_data_sock()
{
    close(sock_data);
    sock_data = -1;
}

int sock_write_data(char *buf, size_t *len)
{
    size_t l = write(sock_data, buf, *len);
    if(l == -1){
        return -1;
    }
    *len = l;
    return 0;
}

int sock_read_data(char *buf, size_t *len)
{
    size_t l = read(sock_data, buf, *len);
    if(l == -1){
        return -1;
    }
    *len = l;
    return 0;
}

int sock_write_cmd(char *cmd_buf)
{
    return write(sock_cmd, cmd_buf, strlen(cmd_buf));
}

int sock_read_cmd(char *cmd_buf,size_t *len)
{
    size_t l = read(sock_cmd, cmd_buf, *len);
    if(l == -1){
        return -1;
    }
    *len = l;
    return 0;
}

int get_ip_address(struct in_addr* ret)
{
    struct ifaddrs *ifaddr, *ifa;
    int family, s, n;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return -1;
    }

    /* Walk through linked list, maintaining head pointer so we
      can free list later */

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        /* For an AF_INET* interface address, display the address */

        if (family == AF_INET/* || family == AF_INET6*/) {
            // s = getnameinfo(ifa->ifa_addr,
            //         (family == AF_INET) ? sizeof(struct sockaddr_in) :
            //                               sizeof(struct sockaddr_in6),
            //         host, NI_MAXHOST,
            //         NULL, 0, NI_NUMERICHOST);
            // if (s != 0) {
            //     printf("getnameinfo() failed: %s\n", gai_strerror(s));
            //     exit(EXIT_FAILURE);
            // }

            // printf("\t\taddress: <%s>\n", host);

            *ret = ((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
        }
    }

    freeifaddrs(ifaddr);
    return 0;
}


/*
int main(int argc, char const *argv[])
{
    char buf[512];
    size_t len;
    init_cmd_sock();
    if(0 != connect_server("cqtest.ddns.net", 21)){
        perror("failed to connect to server");
        exit(1);
    }
    len = sizeof(buf);
    read_cmd(buf, &len);
    buf[len] = 0;
    printf("%s\n", buf);
    close_cmd_sock();
    return 0;
}
*/
