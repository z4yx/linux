#ifndef NETWORK_H__
#define NETWORK_H__

#include <stdio.h>
#include <stdint.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>

void init_cmd_sock();
void close_cmd_sock();
int server_sock_accept(int is_cmd, struct sockaddr_in* client, int *conn_fd);
int server_sock_bind_listen(int is_cmd, short port);
void set_cmd_sock(int fd);
void set_data_sock(int fd);
int connect_server(const char* servername, unsigned short port);
int open_data_sock();
void close_data_sock();
void init_pasv_sock();
void close_pasv_sock();
int connect_data_server(const struct in_addr* data_addr, unsigned short data_port);
int sock_write_cmd(char *cmd_buf);
int sock_read_cmd(char *cmd_buf,size_t *len);
int sock_write_data(char *buf, size_t *len);
int sock_read_data(char *buf, size_t *len);
int get_ip_address(struct in_addr*);

#endif
