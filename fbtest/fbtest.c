#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>

int main()
{
    int fbfd = 0;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize = 0;
    char *fbp = 0;
    int x = 0, y = 0;
    long int location = 0;

    // Open the file for reading and writing
    fbfd = open("/dev/fb0", O_RDWR);
    if (!fbfd) {
        printf("Error: cannot open framebuffer device.\n");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
        printf("Error reading fixed information.\n");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
        printf("Error reading variable information.\n");
        exit(3);
    }

    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fbfd, 0);
    if ((int)fbp == -1) {
        printf("Error: failed to map framebuffer device to memory.\n");
        exit(4);
    }
    printf("The framebuffer device was mapped to memory successfully.\n");

    x = 10; y = 10;       // Where we are going to put the pixel

#define PIX(x,y) ((x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (y+vinfo.yoffset) * finfo.line_length + fbp)

    for(x = 0; x < vinfo.xres; x++)
	*((unsigned short int*)(PIX(x,0))) = 0xFFFF;
    for(x = 0; x < vinfo.xres; x++)
	*((unsigned short int*)(PIX(x,vinfo.yres-1))) = 0xFFFF;
    for(y = 0; y < vinfo.yres; y++)
	*((unsigned short int*)(PIX(0,y))) = 0xFFFF;
    for(y = 0; y < vinfo.yres; y++)
	*((unsigned short int*)(PIX(vinfo.xres-1,y))) = 0xFFFF;

    for(x = 2; x < vinfo.xres-2; x++)
    	for(y = 2; y < vinfo.yres-2; y++)
	    *((unsigned short int*)(PIX(x,y))) = 0x1F;
	    
#if 0
    // Figure out where in memory to put the pixel
    for (y = 10; y < 200; y++)
        for (x = 10; x < 100; x++) {

//            location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
//                       (y+vinfo.yoffset) * finfo.line_length;

            if (vinfo.bits_per_pixel == 32) {
                *(fbp + location) = 100;        // Some blue
                *(fbp + location + 1) = 15+(x-100)/2;     // A little green
                *(fbp + location + 2) = 200-(y-100)/5;    // A lot of red
                *(fbp + location + 3) = 0;      // No transparency
            } else  { //assume 16bpp
                int b = 10;
                int g = (x-100)/6;     // A little green
                int r = 31-(y-100)/16;    // A lot of red
                unsigned short int t = r<<11 | g << 5 | b;
                *((unsigned short int*)(fbp + location)) = t;
                *((unsigned short int*)(PIX(x,y))) = t;
            }

        }
#endif
    munmap(fbp, screensize);
    close(fbfd);
    return 0;
}
