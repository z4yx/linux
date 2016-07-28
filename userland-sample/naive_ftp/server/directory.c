#include "directory.h"
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/param.h>
#include <errno.h>
#include <unistd.h>
#include <dirent.h>

// char *list_directory()
// {
//     struct dirent *ep;
//     DIR *dp = opendir(".");
//     if(dp == NULL) {
//         perror("opendir");
//         return NULL;
//     }
//     ep = readdir(dp);
//     while(ep) {
//         int len = strlen(ep->d_name);

//         ep = readdir(dp);
//     }
//     closedir(dp);

// }

char *list_directory()
{

    FILE *fp;
    int cap = 32, len = 0;
    char buf[64];
    char *output = (char*)malloc(cap);
    if(!output)
        return NULL;

    // setenv("LC_ALL","en_US.UTF-8",1);
    // setenv("LANG","en",1);
    // fp = popen("/usr/local/bin/gls -l", "r");
    fp = popen("/bin/ls -l", "r");
    if (fp == NULL) {
        perror("Failed to ls");
        return NULL;
    }

    /* Read the output a line at a time - output it. */
    while (fgets(buf, sizeof(buf) - 1, fp) != NULL) {
        int tmp = strlen(buf);
        while(len + tmp > cap){
            output = (char*)realloc(output, cap*2);
            if(!output)
                return NULL;
            cap *= 2;
        }
        strcpy(output + len, buf);
        len += tmp;
    }

    /* close */
    pclose(fp);

    return output;
}
/*
int main(int argc, char const *argv[])
{
    char *ret = list_directory();
    printf("%s\n", ret);
    free(ret);
    return 0;
}
*/