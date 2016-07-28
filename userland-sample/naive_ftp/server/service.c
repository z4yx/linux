
#include "service.h"
#include "params.h"
#include "network.h"
#include "directory.h"

#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/param.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>

static char RequestBuf[512];
static char ResponseBuf[512];
static char* RequestArg;

static int Logged, DataConnected;
static enum FtpType CurType;
static enum FtpStru CurStru;
static enum FtpMode CurMode;
static enum DtpType CurDtp;

static struct in_addr ActiveAddr;
static unsigned short ActivePort;

void v_printf(const char* fmt, ...)
{
    if(gIsVerbose){
        va_list args;
        va_start(args, fmt);
        vfprintf(stdout, fmt, args);
        va_end(args);
    }
}

int simple_response(short code, const char* text)
{
    if(text)
        snprintf(ResponseBuf, sizeof(ResponseBuf), "%d %s\r\n", code, text);
    else
        snprintf(ResponseBuf, sizeof(ResponseBuf), "%d\r\n", code);
    int ret = sock_write_cmd(ResponseBuf) == -1 ? -1 : 0;
    ResponseBuf[strlen(ResponseBuf)-2] = '\0';
    v_printf(">%s\n", ResponseBuf);
    return ret;
}

int pathname_response(short code, const char* path)
{
    snprintf(ResponseBuf, sizeof(ResponseBuf), "%d \"%s\"\r\n", code, path);
    int ret = sock_write_cmd(ResponseBuf) == -1 ? -1 : 0;
    ResponseBuf[strlen(ResponseBuf)-2] = '\0';
    v_printf(">%s\n", ResponseBuf);
    return ret;
}

int ip_port_response(short code, uint32_t ip, unsigned short port)
{
    snprintf(ResponseBuf, sizeof(ResponseBuf),
        "%d (%d,%d,%d,%d,%d,%d)\r\n",
        code, ip&0xff, (ip>>8)&0xff, (ip>>16)&0xff, (ip>>24)&0xff,
        (port>>8)&0xff, port&0xff);

    int ret = sock_write_cmd(ResponseBuf) == -1 ? -1 : 0;
    ResponseBuf[strlen(ResponseBuf)-2] = '\0';
    v_printf(">%s\n", ResponseBuf);
    return ret;
}

int read_req_line(void)
{
    size_t i, maxlen = sizeof(RequestBuf)-1;
    for (i = 0; i < maxlen; )
    {
        size_t len = maxlen - i;
        if(-1 == sock_read_cmd(RequestBuf + i, &len)){
            return -1;
        }
        if(len == 0)
            return -1;
        i += len;
        if(RequestBuf[i-1] == '\n')
            break;
    }
    RequestBuf[i-2] = '\0';
    return i-2;
}

char* parse_arguments()
{
    char* space = strchr(RequestBuf, ' ');
    if(!space)
        return NULL;
    *space = '\0';
    do{
        space++;
    }while(' ' == *space);
    if(*space == '\0')
        return NULL;
    char* ret = space;
    while(*space != ' ' && *space != '\0')
        space++;
    *space = '\0';
    return ret;
}

int wait_for_request()
{
    int code;
    int len = read_req_line();
    if(len == -1)
        return -1;
    v_printf("<%s\n", RequestBuf);
    RequestArg = parse_arguments();
    return 0;
}

void init_state()
{
    Logged = 0;
    DataConnected = 0;
    CurStru = STRU_F;
    CurType = TYPE_A;
    CurMode = MODE_S;
    CurDtp = DTP_UNKNOWN;
}

char check_permission()
{
    if(!Logged){
        simple_response(530, "Not logged in.");
        return 0;
    }
    return 1;
}

int retrieve_ip_from_req(const char* resp, struct in_addr *data_addr, unsigned short *data_port)
{
    int h1,h2,h3,h4,p1,p2;
    if(!(*resp) || 6 != sscanf(resp, "%d,%d,%d,%d,%d,%d", &h1,&h2,&h3,&h4,&p1,&p2)){
        printf("%s\n", "Wrong address format");
        return -1;
    }
    data_addr->s_addr = (h4<<24)|(h3<<16)|(h2<<8)|h1;
    *data_port = p2|(p1<<8);
    return 0;
}

int recv_thread(FILE* fp, char* buffer)
{
    int ret = 0;
    if(fp != NULL){
        buffer = (char*)malloc(4096);
        if(!buffer){
            perror("malloc");
            return -1;
        }
    }
    for(;;){
        size_t len = 4096;
        int l = sock_read_data(buffer, &len);
        if(l == -1){
            perror("read");
            ret = -1;
            break;
        }
        // printf("read %d bytes\n", len);
        if(len == 0)
            break;
        if(fp == NULL)
            buffer += len;
        else
            fwrite(buffer, 1, len, fp);
    }
    if(fp != NULL){
        free(buffer);
        fflush(fp);
    }
    // printf("%s quit\n", __func__);
    return ret;
}

int send_thread(FILE* fp, char* buffer, size_t len)
{
    int ret = 0;
    if(fp != NULL){
        buffer = (char*)malloc(4096);
        if(!buffer){
            perror("malloc");
            return -1;
        }
    }
    for(;;){
        if(fp != NULL){
            len = fread(buffer, 1, 4096, fp);
            if(len == 0)
                break;
        }
        for (size_t sent = 0; sent < len; )
        {
            size_t tmp = len - sent;
            // printf("try to send %d bytes\n", tmp);
            if(-1 == sock_write_data(buffer + sent, &tmp)){
                perror("write");
                ret = -1;
                goto out_of_for;
            }
            // printf("%d bytes sent\n", tmp);
            sent += tmp;
        }
        if(fp == NULL)
            break;
    }
out_of_for:
    if(fp != NULL)
        free(buffer);
    // printf("%s quit\n", __func__);
    return ret;
}
    
void deal_with_transfer(FILE *fp, char *buf, size_t size, int isRecv)
{
    int ret;
    if(CurDtp == DTP_ACTIVE){
        if(0 != open_data_sock())
            goto failed;
        if(0 != connect_data_server(&ActiveAddr, ActivePort))
            goto failed;
        DataConnected = 1;
    }else{
        if(!DataConnected)
            goto failed;
    }
    simple_response(125, "Data connection already open; transfer starting.");
    if(isRecv){
        ret = recv_thread(fp, buf);
    }else{
        ret = send_thread(fp, buf, size);
    }
    if(ret == 0){
        simple_response(250, "Completed");
        goto cleanup;
    }
failed:
    simple_response(500, "Internal error");
cleanup:
    close_data_sock();
    DataConnected = 0;
}

void *accept_thread(void* _arg)
{
    struct sockaddr_in client;
    int fd;
    if(0 == server_sock_accept(0, &client, &fd)){
        set_data_sock(fd);
        DataConnected = 1;
    }
    return 0;
}

int switch_dtp(enum DtpType dtp, struct in_addr *addr, unsigned short port)
{
    if(CurDtp == DTP_PASV){
        close_pasv_sock();
    }
    if(dtp == DTP_ACTIVE){
        ActiveAddr = *addr;
        ActivePort = port;
        simple_response(200, "Entering active mode.");
    }else{
        struct in_addr addr;
        unsigned short i;
        pthread_t tid;
        
#if 1
        simple_response(502, "I'm naive");
        return -1;
#else
        init_pasv_sock();
        for (i = DEFAULT_PASV_PORT_START; i < DEFAULT_PASV_PORT_END; ++i)
        {
            if(0 == server_sock_bind_listen(0, i))
                goto listen_succ;
        }
        simple_response(500, "No free port found.");
        return -1;
listen_succ:
        get_ip_address(&addr);
        printf("Passive mode: %s:%d\n", inet_ntoa(addr), i);
        ip_port_response(227, addr.s_addr, i);
        pthread_create(&tid, 0, accept_thread, 0);
#endif
    }
    CurDtp = dtp;
    return 0;
}

void service_handle_client(struct sockaddr_in* client)
{
    printf("New client connected\n");

    init_state();

    simple_response(220, "(Naive FTP Server)");
    for(;wait_for_request() != -1;){
        if(!strcmp(RequestBuf, "TYPE")){
            if(RequestArg){
                if(*RequestArg=='A')
                    CurType = TYPE_A;
                else if(*RequestArg=='I')
                    CurType = TYPE_I;
                simple_response(200, "TYPE set");
            }else{
                simple_response(501, "need argument");
            }
        }else if(!strcmp(RequestBuf, "MODE")){
            simple_response(200, "MODE set");
        }else if(!strcmp(RequestBuf, "LIST")){
            if(!check_permission())
                continue;
            char *content = list_directory();
            deal_with_transfer(NULL, content, strlen(content), 0);
            free(content);
        }else if(!strcmp(RequestBuf, "PWD")){
            if(!check_permission())
                continue;
            char cwd[MAXPATHLEN+1];
            getwd(cwd);
            pathname_response(257, cwd);
        }else if(!strcmp(RequestBuf, "CWD")){
            if(!check_permission())
                continue;
            if(-1 == chdir(RequestArg))
                simple_response(550, "Requested action not taken.");
            else
                simple_response(250, "Directory changed.");
        }else if(!strcmp(RequestBuf, "STRU")){
            simple_response(200, "STRU set");
        }else if(!strcmp(RequestBuf, "USER")){
            if(!strcmp(RequestArg, DEFAULT_USER)){
                Logged = 1;
                simple_response(230, "User logged in, proceed.");
            }else{
                simple_response(530, "Wrong user name.");
            }
        }else if(!strcmp(RequestBuf, "QUIT")){
            simple_response(221, "Bye");
            break;
        }else if(!strcmp(RequestBuf, "PASV")){
            if(!check_permission())
                continue;
            switch_dtp(DTP_PASV, NULL, 0);
        }else if(!strcmp(RequestBuf, "PORT")){
            if(!check_permission())
                continue;
            struct in_addr addr;
            unsigned short port;
            if(-1 == retrieve_ip_from_req(RequestArg, &addr, &port)){
                simple_response(501, "Syntax error");
                continue;
            }
            switch_dtp(DTP_ACTIVE, &addr, port);
        }else if(!strcmp(RequestBuf, "NOOP")){
            simple_response(200, "OK");
        }else if(!strcmp(RequestBuf, "RETR")){
            if(!check_permission())
                continue;
            FILE* fp = fopen(RequestArg, "r");
            if(!fp){
                simple_response(550, "Cannot open for reading");
                continue;
            }
            deal_with_transfer(fp, NULL, 0, 0);
            fclose(fp);
        }else if(!strcmp(RequestBuf, "STOR")){
            if(!check_permission())
                continue;
            FILE* fp = fopen(RequestArg, "w");
            if(!fp){
                simple_response(550, "Cannot open for writing");
                continue;
            }
            deal_with_transfer(fp, NULL, 0, 1);
            fclose(fp);
        }else{
            simple_response(502, "I'm naive");
        }
    }
    close_cmd_sock();
    close_pasv_sock();
    printf("Client Disconnected\n");
}


