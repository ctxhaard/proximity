#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>

#define OUT_LEN (5)

#define xstr(a) str(a)
#define str(a) #a

static void do_main(int num)
{
	int fd_lcd;
	int fd_prox;
	int pos;
	char prox_name[11] = { 0x00 };

	pos = 16 + (num * 6);

	//srand(pos);

	printf("task: %d opening %s\n", num, xstr(DEV));

	fd_lcd = open(xstr(DEV), O_WRONLY);
	if(fd_lcd < 0) {
		perror("cannot open LCD device\n");
		return;
	}

	sprintf(prox_name, "/dev/proximity%d", num);
	printf("task: %d opening %s\n", num, prox_name);
	fd_prox = open(prox_name, O_RDONLY);
	if (fd_prox < 0) {
		perror("cannot open proximity device\n");
		return;
	}

	do {
		char buffer[OUT_LEN];
		int val;
		int len;

		if (lseek(fd_lcd, pos, SEEK_SET) < 0) {
			perror("lseek error\n");
			return;
		}

		len = read(fd_prox, buffer, OUT_LEN - 1);
		if (len < 0) {
			perror("read error\n");
			return;
		}
		buffer[len] = '\0';

		val = atoi(buffer);
		val /= 10;

		sprintf(buffer, "%*d", (OUT_LEN-1), val);

		if (write(fd_lcd,buffer,(OUT_LEN-1)) < 0) {
			perror("write error\n");
			return;
		}
		//printf("task: %d => wrote [ %s ] on pos [ %d ]\n",num , buffer, pos);
	}
	while(1);
	close(fd_prox);
	close(fd_lcd);
}

int main(int argc, const char *argv[])
{
	int n_childs = 0;
	pid_t pid;
	int fd_lcd;
	
	printf("#################\nLCD test x86\n#################\n");
	if (argc > 1) {
		n_childs = atoi(argv[1]);
	}

	fd_lcd = open(xstr(DEV), O_WRONLY);
	if(fd_lcd < 0) {
		perror("cannot open device\n");
		return -1;
	}

	if (lseek(fd_lcd, 2, SEEK_SET) < 0) {
		perror("lseek error\n");
		return -1;
	}

	if (write(fd_lcd,"Distance (cm)",13) < 0) {
		perror("write error\n");
		return -1;
	}

	close(fd_lcd);


	while (n_childs) {
		pid = fork();
		if (!pid) {
			/* child */
			do_main(n_childs);
		} else if(pid < 0) {
			perror("error forking\n");
		}
		--n_childs;
	}
	do_main(0);
	wait(NULL);

	return 0;
}


