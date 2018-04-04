#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

/*IOCTL MICRO*/
#define RFID_MAGIC  'R'
#define DUMP       _IOW(RFID_MAGIC, 1, int)

#define DRIVE_NAME "RFID_drv"
#define Device_path "/dev/RFID_drv"
#define err_handler(en,msg) do{errno=en;perror(msg);exit(EXIT_SUCCESS);}while(0) 
int main(){
	int fd,ret,ch,i;
	fd=open(Device_path,O_RDWR);
	if(fd<0)
		err_handler(fd,"open");
	while(1){
		printf("Enter the choice\n");
		printf("-------------------\n");
		printf("1.Scan\n");
		printf("2.EXIT\n");
		printf("-------------------\n");
		scanf("%d",&ch);
		switch(ch){
			case 1:
				ret=ioctl(fd,DUMP,1);
				if(ret<0)
					err_handler(fd,"ioctl");
				break;
			case 2: 
				exit(1);
				
		}
	}
	return 0;
}
