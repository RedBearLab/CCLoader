#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

int RS232_OpenComport(int, int);
int RS232_PollComport(int, unsigned char *, int);
int RS232_ReadBlock(int, unsigned char *, size_t, int);
int RS232_SendByte(int, unsigned char);
int RS232_SendBuf(int, unsigned char *, int);
void RS232_CloseComport(int);
void RS232_cputs(int, const char *);
int RS232_IsCTSEnabled(int);
int RS232_IsDSREnabled(int);
void RS232_enableDTR(int);
void RS232_disableDTR(int);
void RS232_enableRTS(int);
void RS232_disableRTS(int);

#define SBEGIN  0x01
#define SDATA   0x02
#define SRSP    0x03
#define SEND    0x04
#define ERRO    0x05
#define CHIP_ID 0x11
#define SDUMP   0x12
#define FBLOCK  0x13
  

FILE *pfile = NULL;
long fsize = 0;
int BlkTot = 0;
int Remain = 0;
int BlkNum = 0;
int DownloadProgress = 0;
int com = -1;
int end = 0;

int Cport[30], error;

struct termios new_port_settings, old_port_settings[30];
const char* comports[2] = { NULL, NULL };

  
void ProcessProgram(int cmd);

/*
* argv[0]----.exe file name
* argv[1]----command: id, read, write
* argv[2]----serial device
* argv[3]----file path
* argv[4]----device type
* argv[5]----verify mode
*/

#define CMD_ID 1
#define CMD_READ 2
#define CMD_WRITE 3


void usage_help(const char *prgname)
{
  printf("Invalid parameters.\n");
  printf("Usage: %s <serialport> <device> <command> [<bin file> [<verify> | <read_start_block> <read_blocks>]]\n",
	 prgname);
  printf("Examples:\n");
  printf("Read Chip ID: %s /dev/ttyUSB0 1 id\n", prgname);
  printf(" Write Flash: %s /dev/ttyUSB0 1 write flash.bin 1\n", prgname);
  printf("  Read Flash: %s /dev/ttyUSB0 1 read dump.bin 0 512\n", prgname);
  printf("Example: %s /dev/ttyUSB0 1 id\n", prgname);
  printf("          <device>: 0 -- Default (e.g. UNO)\n");
  printf("                    1 -- Leonardo/Mini Pro/etc...\n");
  printf("          <verify>: 0 -- No verify (default)\n");
  printf("                    1 -- Verify (when flashing)\n");
  printf(" <read_stat_block>: 0 -- Start flash dump from block (0 = beginning of the flash)\n");
  printf("     <read_blocks>: 512 -- How many blocks to read (512 = 256Kb)\n");
  
  exit(1);
}

  
int main(int arg, char *argv[])
{	
  int device = 0;
  unsigned char cmd_buf[5] = { 0,0,0,0,0 };
  int verify = 0;

  const char *serialport = argv[1];
  const char *device_arg = argv[2];
  const char *command = argv[3];
  const char *filename = argv[4];
  const char *verify_arg = argv[5];
  const char *size_arg = argv[6];
  
  
  if (arg <=3) usage_help(argv[0]);
  verify=atoi(verify_arg);
  
  if (strncmp(command,"id",2)==0) {
    cmd_buf[0]=CHIP_ID;
    filename="/dev/null";
  }
  else if (strncmp(command,"read",4)==0) {
    int read_size = 0;
    int start_block = 0;
    
    if (arg <= 6) usage_help(argv[0]);
    read_size = atoi(size_arg);
    if (read_size < 1 || read_size > 1024) {
      printf("Invalid number of blocks to read  specified: %d\n",read_size);
      return 1;
    }
    start_block=verify;
    if (start_block < 0 || start_block > 1024) {
      printf("Invalid starting block specified: %d\n", start_block);
      return 1;
    }
    BlkTot=read_size;
    fsize=BlkTot*512;
    cmd_buf[0]=SDUMP;
    cmd_buf[1]=(BlkTot >> 8) & 0xff;
    cmd_buf[2]=BlkTot & 0xff;
    cmd_buf[3]=(start_block >> 8) & 0xff;
    cmd_buf[4]=start_block & 0xff;
  }
  else if (strncmp(command,"write",5)==0) {
    if (arg <= 5) usage_help(argv[0]);
    cmd_buf[0]=SBEGIN;
    cmd_buf[1]=(verify > 0 ? 1 : 0);
    if (verify > 0) {
      printf("Verify enabled (flashing process will take longer)\n");
    }
  }
  else {
    printf("Invalid command: %s\n",command);
    return 1;
  }


  if (!serialport || strlen(serialport) < 1) {
    printf("Missing serial port parameter\n");
    return 1;
  }
  // dynamically generate entry in comports table...
  com = 0;
  comports[com]=serialport;
   
  if(1 == RS232_OpenComport(com, 115200)) {
    return 1;	// Open comprt error
  }
  printf("Serial port: %s\n",serialport);

  device = atoi(device_arg);
  if(device == 0) {
    printf("Device  : Default (e.g. UNO)\n");
    printf("Baud:115200 data:8 parity:none stopbit:1 DTR:off RTS:off\n");
    RS232_disableDTR(com);
  }  else {
    printf("Device: Leonardo\n");
    printf("Baud:115200 data:8 parity:none stopbit:1 DTR:on RTS:off\n");
    RS232_enableDTR(com);
  }
  RS232_disableRTS(com);

  
  if (strlen(filename) < 1) {
    printf("invalid filename: %s\n",filename);
    return 1;
  }

  if (BlkTot > 0) {
    // Read mode so create new file for to save flas into...
    pfile = fopen(filename, "wb");       // create new file
    if(NULL == pfile) {
      printf("cannot create file: %s (%d)\n",filename,errno);
      return 1;
    }
  } else {
    pfile = fopen(filename, "rb");      // read only
    if(NULL == pfile) {
      printf("cannot open file: %s (%d)\n",filename,errno);
      return 1;
    }
    
    fseek(pfile,0,SEEK_SET);
    fseek(pfile,0,SEEK_END);
    fsize = ftell(pfile);
    fseek(pfile,0,SEEK_SET);
    if (fsize < 1 && cmd_buf[0] != CHIP_ID) {
      printf("Cannot flash empty file!\n");
      return 1;
    }
    //BlkTot = fsize / 512;
    Remain = fsize % 512;
    if(Remain != 0) {
      BlkTot = fsize / 512 + 1;
      printf("Warning: file size isn't the integer multiples of 512, last bytes will be set to 0xFF\n");
    } else {
      BlkTot = fsize / 512;
    }
  }

  if (cmd_buf[0] != CHIP_ID) {
    printf("Image file: %s\n",filename);
    printf("Total blocks: %d (%ld bytes)\n", BlkTot, fsize);
  }
  BlkNum = 0;


  printf("\nSend command: %s\n",command);
  //fprintf(stderr, "SEND: %02x %02x %02x\n", cmd_buf[0], cmd_buf[1], cmd_buf[2]);
  
  if(RS232_SendBuf(com, cmd_buf, sizeof(cmd_buf)) != sizeof(cmd_buf)) {
    printf("Communication failure!\n");
    fclose(pfile);
    RS232_CloseComport(com);
    return 1;
  }
  
  printf("Waiting for response...\n\n");
	
  while(!end) {
    ProcessProgram(cmd_buf[0]);
  }
  printf("\nClosing serial port\n");
  BlkNum = 0;
  DownloadProgress = 0;
  fclose(pfile);
  RS232_CloseComport(com);
  
  return 0;
}

void ProcessProgram(int cmd)
{
  int len;
  unsigned char rx;
  len = RS232_PollComport(com, &rx, 1);
  if(len > 0) {
    //fprintf(stderr,"READ: %02x\n", rx);
    
    switch(rx) {

    case CHIP_ID:
      {
	unsigned char buf[2];
	len = RS232_ReadBlock(com, buf, 2, 5);
	if (len < 2) {
	  end = 1;
	  printf("Did not receive chip ID (%d)\n", len);
	} else {
	  printf("      Chip ID: 0x%x (CC25%02x)\n",buf[0],buf[0]);
	  printf("Chip Revision: 0x%x\n\n",buf[1]);
	  if (cmd == CHIP_ID) {
	    end = 1;
	  }
	}
      }
      break;

    case FBLOCK:
      {
	unsigned char buf[514];
	size_t len;
	unsigned short checksum1 = 0;
	unsigned short checksum2 = 0;
	int i;

	memset(buf,0,sizeof(buf));
	BlkNum++;
	if (BlkNum == 1)
	  printf("Reading flash...");
	printf(" %d",BlkNum);
	fflush(stdout);

	len = RS232_ReadBlock(com, buf, 514, 10);
	for (i=0; i<512; i++) {
	  checksum1 += buf[i];
	}
	checksum2 = (buf[512]<<8 | buf[513]);
	if (checksum1 != checksum2) {
	  printf("\nBlock %d: checksum mismatch: %04x vs %04x\n",BlkNum,checksum1,checksum2);
	}
	
	len=fwrite(buf,1,512,pfile);
	if (len < 512) {
	  printf("failed to write block to file: %d\n",errno);
	  exit(1);
	}

	if (BlkNum >= BlkTot) {
	  printf("\nFlash Dump Complete\n");
	  end = 1;
	  break;
	}
	
      }
      break;
      
    case SRSP:
      {
	if (BlkNum == BlkTot) {
	  unsigned char temp = SEND;
	  RS232_SendByte(com, temp);
	  end = 1;
	  printf("\nEnd programming\n");
	} else  {
	  if(BlkNum == 0) {	
	    printf("Begin programming...\n");
	  }
	  DownloadProgress = 1;
	  unsigned char buf[515];
	  buf[0] = SDATA;
		
	  if((BlkNum == (BlkTot-1)) && (Remain != 0)) {
	    fread(buf+1, Remain, 1, pfile);
	    int filled = 512 - Remain;
	    //int i = 0;
	    for(int i = 0; i<filled; i++) {
	      buf[Remain+1+i] = 0xFF;
	    }
	  } else {
	    fread(buf+1, 512, 1, pfile);
	  }
                    

	  unsigned short CheckSum = 0x0000;
	  //unsigned int i;
	  for(unsigned int i=0; i<512; i++) {
	    CheckSum += (unsigned char)buf[i+1];
	  }
	  buf[513] = (CheckSum >> 8) & 0x00FF;
	  buf[514] = CheckSum & 0x00FF;
                    
	  RS232_SendBuf(com, buf, 515);
	  BlkNum++;
	  printf("%d  ", BlkNum);
	  fflush(stdout);
	}
      }
      break;

    case ERRO:
      {
	if(DownloadProgress == 1) {
	  end = 1;
	  printf("Verify failed!\n");
	} else {
	  end = 1;
	  printf("No chip detected!\n");
	}
      }
      break;

    default:
      printf("Unknown response: %x\n",rx);
      break;
    }
    
    len = 0;
  }
  
}

int RS232_OpenComport(int comport_number, int baudrate)
{
  int baudr, status;

  if((comport_number>29)||(comport_number<0))
  {
    printf("illegal comport number\n");
    return(1);
  }

  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  Cport[comport_number] = open(comports[comport_number], O_RDWR | O_NOCTTY | O_NDELAY);
  if(Cport[comport_number]==-1)
  {
    perror("unable to open comport ");
    return(1);
  }

  error = tcgetattr(Cport[comport_number], old_port_settings + comport_number);
  if(error==-1)
  {
    close(Cport[comport_number]);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
  new_port_settings.c_iflag = IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */
  error = tcsetattr(Cport[comport_number], TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(Cport[comport_number]);
    perror("unable to adjust portsettings ");
    return(1);
  }

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
    return(1);
  }

  return(0);
}


int RS232_PollComport(int comport_number, unsigned char *buf, int size)
{
  int n;

#ifndef __STRICT_ANSI__                       /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
  if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
#else
  if(size>4096)  size = 4096;
#endif

  n = read(Cport[comport_number], buf, size);

  return(n);
}

int RS232_ReadBlock(int comport_number, unsigned char *buf, size_t size, int timeout)
{

  size_t read_left = size;
  size_t len;
  time_t t_start = time(NULL);
  
  while (read_left > 0) {
    do {
      len = read(Cport[comport_number], &buf[size-read_left], read_left);
    } while (len < 0 && errno==EINTR);
    if (len < 0) {
      printf("error reading serial port: %d (%d)\n",len,errno);
      exit(1);
    }
    if (timeout > 0) {
      if ((time(NULL)-t_start) > timeout) {
	printf("timeout reading serial port\n");
	exit(2);
      }
    }
    read_left-=len;
  }

  return size;
}

int RS232_SendByte(int comport_number, unsigned char byte)
{
  int n;

  n = write(Cport[comport_number], &byte, 1);
  if(n<0)  return(1);

  return(0);
}


int RS232_SendBuf(int comport_number, unsigned char *buf, int size)
{
  return(write(Cport[comport_number], buf, size));
}


void RS232_CloseComport(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */
  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }

  close(Cport[comport_number]);
  tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
}

/*
Constant  Description
TIOCM_LE  DSR (data set ready/line enable)
TIOCM_DTR DTR (data terminal ready)
TIOCM_RTS RTS (request to send)
TIOCM_ST  Secondary TXD (transmit)
TIOCM_SR  Secondary RXD (receive)
TIOCM_CTS CTS (clear to send)
TIOCM_CAR DCD (data carrier detect)
TIOCM_CD  Synonym for TIOCM_CAR
TIOCM_RNG RNG (ring)
TIOCM_RI  Synonym for TIOCM_RNG
TIOCM_DSR DSR (data set ready)
*/

int RS232_IsCTSEnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_CTS) return(1);
  else return(0);
}

int RS232_IsDSREnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_DSR) return(1);
  else return(0);
}

void RS232_enableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_enableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

