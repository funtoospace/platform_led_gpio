#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

/*   platform_led id on
  *  platform_led id off
  *  id: 1,2,3,4
  */
int main(int argc, char **argv)
{
	int fd;
	int state = 0;
	int val = 1;
	//fd_set rfds;
	struct epoll_event ev_rd_platform_led;
	int err;
	int epfd;

	fd = open("/dev/s3c24xx_led", O_RDWR | O_NONBLOCK);
	if (fd < 0)
	{
		printf("/dev/s3c24xx_led can't open!\n");
	}
	if (argc != 3)
	{
		printf("Usage :\n");
		printf("%s id <on|off>\n", argv[0]);
		return 0;
	}


	if (strcmp(argv[2], "on") == 0)
	{
		if (strcmp(argv[1], "1") == 0)
			val  = 11;
		else if (strcmp(argv[1], "2") == 0)
			val  = 21;
		else if (strcmp(argv[1], "3") == 0)
			val  = 31;
		else if (strcmp(argv[1], "4") == 0)
			val  = 41;		
		
		write(fd, &val, 4);
	}
	else if (strcmp(argv[2], "off") == 0)
	{
		if (strcmp(argv[1], "1") == 0)
			val  = 10;
		else if (strcmp(argv[1], "2") == 0)
			val  = 20;
		else if (strcmp(argv[1], "3") == 0)
			val  = 30;
		else if (strcmp(argv[1], "4") == 0)
			val  = 40;	
	
		write(fd, &val, 4);
	}
	/*  
	else if (strcmp(argv[1], "read") == 0)
	{
		//FD_ZERO(&rfds);
		//FD_SET(fd, &rfds);

		//select(fd + 1, &rfds, NULL, NULL, NULL);

		//if (FD_ISSET(fd, &rfds))
		//	printf("Poll monitor: platform_led is on, it can be read\n");
		//else
		//	printf("Poll monitor: platform_led is off, enter wait-queue\n");
		epfd = epoll_create(1);
		if(epfd  < 0) {
			perror("epoll_create error");
			return;
		}

		bzero(&ev_rd_platform_led, sizeof(struct epoll_event));
		ev_rd_platform_led.events = EPOLLIN;

		err = epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev_rd_platform_led);
		if (err < 0) {
			perror("epoll_ctl: add_ev error");
			return;
		}

		err = epoll_wait(epfd, &ev_rd_platform_led, 1, 10000);
		if (err < 0) {
			perror("epoll_wait error");
		} else if (err == 0) {
			printf("Epoll monitor: platform_led isn't set ON within 10 seconds\n");
		} else {
			printf("Epoll monitor: platform_led can be read\n");
		}

		err = epoll_ctl(epfd, EPOLL_CTL_DEL, fd, &ev_rd_platform_led);
		if (err < 0)
			perror("epoll_del: del_ev error");

		read(fd, &state, 4);
		if (state == 1)
			printf("platform_led is on, state = %d\n",state);
		else
			printf("platform_led is off, state = %d\n", state);
	}
	*/
	
	return 0;
}
