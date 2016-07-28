#ifndef __SERVICE_H_
#define __SERVICE_H_

#include <netinet/in.h>

enum FtpType{TYPE_A,TYPE_E,TYPE_I,TYPE_L,TYPE_N,TYPE_T,TYPE_C};
enum FtpStru{STRU_F, STRU_R, STRU_P};
enum FtpMode{MODE_S, MODE_B, MODE_C};
enum DtpType{DTP_UNKNOWN, DTP_ACTIVE, DTP_PASV};

void service_handle_client(struct sockaddr_in* client);

#endif