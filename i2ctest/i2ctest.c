#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
 
#define I2C_SLAVE 0x0703

int setreg(int file, __u8 reg, __u8 val)
{
	char buf[2];
	buf[0] = reg;
	buf[1] = val;
	
	int result = write(file,buf,2);
	if(result != 2)
	{
		printf("write reg %d fail, result=%d\n",reg, result);
	}
}

__u8 getreg(int file, __u8 reg)
{
	char buf[2];
	buf[0] = reg;
	int result = write(file,buf,1);
	buf[0] = 0;
	result = read(file,buf,1);
	return buf[0];
}

int main()
{
	int file;
	int adapter_nr = 0;
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	file = open(filename, O_RDWR);
	if (file < 0)
	{
		printf("cannot open filen\n");
		exit(1);
	}

	printf("file opened\n");

	int addr = 0x20; 
	if(ioctl(file, I2C_SLAVE, addr) < 0)
	{
		printf("cannot find slave device address 0x%x\n", addr);
		exit(1);
	}

	setreg(file, 3,0xF0);
	setreg(file, 2,0x00);

	int i;
	__u8 val = 1;
	while(1)
	{
		setreg(file, 1,(~val)&0xFF);
		sleep(1);
		val<<=1;
		if(val==0x10)
			val = 1;
		int k = getreg(file,0);
		if((k&0x10) == 0)
			break;
	}


	printf("reg%d=0x%x\n", 0, getreg(file, 0));	


	close(file);
}

