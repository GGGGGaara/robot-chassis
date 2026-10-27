/* Test-only LD_PRELOAD fault injector. It can affect vcan_navtest only. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <linux/can.h>
#include <net/if.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

ssize_t write(int fd, const void *data, size_t length) {
  static ssize_t (*real_write)(int,const void*,size_t);
  if (!real_write) real_write=dlsym(RTLD_NEXT,"write");
  const char *flag=getenv("TRACER_TEST_CAN_FAIL_FILE");
  if (flag && length==sizeof(struct can_frame) && access(flag,F_OK)==0) {
    struct sockaddr_can address; socklen_t size=sizeof(address); char name[IF_NAMESIZE];
    if (getsockname(fd,(struct sockaddr*)&address,&size)==0 && address.can_family==AF_CAN &&
        if_indextoname(address.can_ifindex,name) && strcmp(name,"vcan_navtest")==0) {
      const struct can_frame *frame=data;
      if (frame->can_id==0x111) {errno=ENOBUFS; return -1;}
    }
  }
  return real_write(fd,data,length);
}
