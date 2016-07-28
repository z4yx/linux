#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>

int main(int argc, char *argv[])
{
    // mkdir("/dev",0777);
    // mknod("/dev/console",0777,makedev(5,1));
    // freopen("/dev/console","w",stdout);
    // FILE *f = fopen("/dev/console","w");
    // if(!f) return 1;
    struct stat result;
    int ret;
    // int ret = stat("/dev/console", &result);
    // if(ENOENT == ret)
    //     return 2;
    // if(ret)
    //     return 3;
    ret = open("/dev/console", O_WRONLY);
    if(ret<0)
        return errno;

printf("Hello world!\n");

sleep(999999999);
return 0l;
}
