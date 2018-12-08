#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(_WIN64)
#elif defined(__linux__) || defined(__unix__)
#elif defined(__APPLE__) || defined(__MACH__)
#endif


#define _BSD_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
#elif defined(__linux__) || defined(__unix__)
  #include <termios.h>
  #include <sys/ioctl.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <limits.h>
#endif

#define SBEGIN  0x01
#define SDATA   0x02
#define SRSP    0x03
#define SEND    0x04
#define ERRO    0x05

#define N_TRIES    5

FILE *pfile = NULL;
long fsize = 0;
int BlkTot = 0;
int Remain = 0;
int BlkNum = 0;
int DownloadProgress = 0;
int com = -1;
int tries = N_TRIES;
int end = 0;

#if defined(_WIN32) || defined(_WIN64)
HANDLE Cport[16];
char comports[16][10]={"\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
                       "\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
                       "\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
                       "\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16"};
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
int Cport, error;
struct termios new_port_settings, old_port_settings;
char comports[30][16]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS5",
                       "/dev/ttyS6","/dev/ttyS7","/dev/ttyS8","/dev/ttyS9","/dev/ttyS10","/dev/ttyS11",
                       "/dev/ttyS12","/dev/ttyS13","/dev/ttyS14","/dev/ttyS15","/dev/ttyUSB0",
                       "/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",
                       "/dev/ttyAMA0","/dev/ttyAMA1","/dev/ttyACM0","/dev/ttyACM1",
                       "/dev/rfcomm0","/dev/rfcomm1","/dev/ircomm0","/dev/ircomm1"};
#endif

void ProcessProgram(void);

int RS232_OpenComport(int, int);
int RS232_PollComport(int, unsigned char *, int);
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

/*
* argv[0]----.exe file name
* argv[1]----ComPort number
* argv[2]----file path
* argv[3]----Device
*/
int main(int arg, char *argv[])
{
	int fLen = 0;
	int device = 0;

	printf("Copyright (c) 2013 RedBearLab.com\n");
	printf("%s version 0.7\n", argv[0]);
	if(arg < 4)
	{
		printf("Invalid parameters.\n");
		printf("Usage: %s <com number> <bin file> <device>\n", argv[0]);
		printf("Example: %s 2 abc.bin 0\n", argv[0]);
		printf(" <device>: 0 -- Default (e.g. UNO)\n");
		printf("           1 -- Leonardo\n\n");
		return 0;
	}


    /// Open the file
    char form[5] = ".bin";
    char format[5] = "    ";
    fLen = strlen(argv[2]);
    if(fLen < 5)
    {
        printf("The .bin file name is invalid!\n\n");
        return 0;  // file path is not valid
    }
    format[3] = argv[2][fLen-1];
    format[2] = argv[2][fLen-2];
    format[1] = argv[2][fLen-3];
    format[0] = argv[2][fLen-4];
    if(0 != strcmp(form, format))
    {
        printf("File format must be .bin\n\n");
        return 0;
    }
    pfile = fopen(argv[2], "rb");      // read only
    if(NULL == pfile)
    {
        printf("The file doesn't exist or is occupied!\n\n");
        return 0;
    }
    printf("Bin file: %s ,\t", argv[2]);
    printf(" file open successfully!\n");
    fseek(pfile,0,SEEK_SET);
    fseek(pfile,0,SEEK_END);
    fsize = ftell(pfile);
    fseek(pfile,0,SEEK_SET);
    Remain = fsize % 512;
    if(Remain != 0)
    {
        BlkTot = fsize / 512 + 1;
        printf("!!WARNING: File's size isn't the integer multiples of 512 bytes, and \n");
        printf("           the last block will be filled in up to 512 bytes with 0xFF! \n");
    }
    else
    {
        BlkTot = fsize / 512;
    }
    printf("Block total: %d\n", BlkTot);
    BlkNum = 0;


    /// Open serial comPort
    device = atoi(argv[3]);
    com = atoi(argv[1]) - 1;
#if defined(_WIN32) || defined(_WIN64)
    printf("ComPort: COM%d ", com+1);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
    for(int i=0; i<30; i++)
    {
        if(NULL != strstr(comports[i], argv[1]))
        {
            com = i;
            break;
        }
        else if(i == 29 && com == -1)
        {
            printf("Port not found, check the arguments.\n");
            return 0;
        }
    }
    printf("ComPort: %s ", comports[com]);
#endif
    if(1 == RS232_OpenComport(com, 115200))
        return 0;	// Open comport error
    printf("open successfully.\n");

    printf("Waiting for Arduino setup...\n");

    printf("Device: ");
    if(device == 0)
    {
        printf("Default (e.g. UNO)\n");
        RS232_disableDTR(com);
    }
    else
    {
        printf("Leonardo\n");
        RS232_enableDTR(com);
    }
    printf("<Baud:115200> <data:8> <parity:none> <stopbit:1> <DTR:on> <RTS:off>\n");
    RS232_disableRTS(com);
#if defined(__APPLE__) || defined(__MACH__) || defined(__linux__) || defined(__unix__)
    // I don't know why, but I need this.
    sleep(2); // slow down @Linux
#endif

    printf("Enable transmission...");
    unsigned char buf[2] = {SBEGIN, 0};      // Enable transmission,  do not verify
    if(RS232_SendBuf(com, buf, 2) != 2)
    {
        printf("Enable failed!\n");
        fclose(pfile);
        printf("File closed!\t");
        RS232_CloseComport(com);
        printf("Comport closed!\n\n");
        return 0;
    }
    else
    {
        printf("Request sent already!\n");
    }
#if defined(__APPLE__) || defined(__MACH__) || defined(__linux__) || defined(__unix__)
    usleep(5e5); // 0.1 seconds - delay to wait transmission send
#endif
    
    // Replaced by !tries--
    //printf("/********************************************************************/\n");
    //printf("* If there is no respond last for 3s, please press \"Ctrl+C\" to exit!\n");
    //printf("* And pay attention to :\n");
    //printf("*   1. The connection between computer and Arduino;\n");
    //printf("*   2. The connection between Arduino and CC254x;\n");
    //printf("*   3. Whether the device you using is Leonardo or not;\n");
    //printf("*   4. Other unexpected errors.\n");
    //printf("/********************************************************************/\n\n");

    printf("Waiting for respond from Arduino...");
    while(!end)
    {
        ProcessProgram();
    }

    if(end != 2)
        printf("Upload Failed!\n");

    BlkNum = 0;
    DownloadProgress = 0;
    fclose(pfile);
    printf("File closed!\n");
    RS232_CloseComport(com);
    printf("Comport closed!\n\n");

    return 0;
}

void ProcessProgram()
{
    int len;
    unsigned char rx;
    len = RS232_PollComport(com, &rx, 1);
#if defined(__APPLE__) || defined(__MACH__) || defined(__linux__) || defined(__unix__)
    usleep(1e5); // slow down @Linux
#endif
    if(len > 0)
    {
        switch(rx)
        {
            case SRSP:
            {
                if(BlkNum == BlkTot)
                {
                    unsigned char temp = SEND;
                    RS232_SendByte(com, temp);
                    printf("Upload successfully!\n");
                    end = 2;
                }
                else
                {
                    if(BlkNum == 0)
                    {
                        printf("OK.\nUploading firmware.../n");
                    }
                    DownloadProgress = 1;
                    unsigned char buf[515];
                    buf[0] = SDATA;
                    if((BlkNum == (BlkTot-1)) && (Remain != 0))
                    {
                        fread(buf+1, Remain, 1, pfile);
                        int filled = 512 - Remain;
                        for(int i = 0; i<filled; i++)
                        {
                            buf[Remain+1+i] = 0xFF;
                        }
                    }
                    else
                    {
                        fread(buf+1, 512, 1, pfile);
                    }
                    
                    unsigned short CheckSum = 0x0000;
                    for(unsigned int i=0; i<512; i++)
                    {
                        CheckSum += (unsigned char)buf[i+1];
                    }
                    buf[513] = (CheckSum >> 8) & 0x00FF;
                    buf[514] = CheckSum & 0x00FF;
                    RS232_SendBuf(com, buf, 515);
                    BlkNum++;
                    printf("%d\t    \r", BlkNum);
                }
                break;
            }

            case ERRO:
            {
                if(DownloadProgress == 1)
                {
                    end = 1;
                    printf("Verify failed!\n");
                }
                else
                {
                    end = 1;
                    printf("No chip detected!\n");
                }
                break;
            }

            default:
            {
                printf("Something wrong happened with the Arduino answer's!\n");
                break;
            }
        }
        len = 0;
    }
    else
    {
        printf("try:%d  ", N_TRIES+1-tries);
        if( !tries-- )
        {
            end = 1;
            printf("I tried with no success!\n");
        }
#if defined(__APPLE__) || defined(__MACH__) || defined(__linux__) || defined(__unix__)
        else
        usleep(1e5); // 0.1 second
#endif
    }
}

int RS232_OpenComport(int comport_number, int baudrate)
{
#if defined(_WIN32) || defined(_WIN64)
  char baudr[64];
  if((comport_number>15)||(comport_number<0))
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int baudr, status;
  if((comport_number>29)||(comport_number<0))
#endif
  {
    printf("Illegal comport number\n\n");
    return(1);
  }

  switch(baudrate)
  {
#if defined(_WIN32) || defined(_WIN64)
    case     110 : strcpy(baudr, "baud=110 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case     300 : strcpy(baudr, "baud=300 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case     600 : strcpy(baudr, "baud=600 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    1200 : strcpy(baudr, "baud=1200 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    2400 : strcpy(baudr, "baud=2400 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    4800 : strcpy(baudr, "baud=4800 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    9600 : strcpy(baudr, "baud=9600 data=8 parity=N stop=1 dtr=off rts=off");
                   break;
    case   19200 : strcpy(baudr, "baud=19200 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case   38400 : strcpy(baudr, "baud=38400 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case   57600 : strcpy(baudr, "baud=57600 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  115200 : strcpy(baudr, "baud=115200 data=8 parity=N stop=1 dtr=off rts=off");
                   break;
    case  128000 : strcpy(baudr, "baud=128000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  256000 : strcpy(baudr, "baud=256000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  500000 : strcpy(baudr, "baud=500000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case 1000000 : strcpy(baudr, "baud=1000000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    default      : printf("Invalid baudrate\n");
                   return(1);
                   break;
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
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
    default      : printf("Invalid baudrate\n");
                   return(1);
                   break;
#endif
	}

#if defined(_WIN32) || defined(_WIN64)
  Cport[comport_number] = CreateFileA(comports[comport_number],
                      GENERIC_READ|GENERIC_WRITE,
                      0,                          /* no share  */
                      NULL,                       /* no security */
                      OPEN_EXISTING,
                      0,                          /* no threads */
                      NULL);                      /* no templates */

  if(Cport[comport_number]==INVALID_HANDLE_VALUE)
  {
    printf("unable to open comport!\n\n");
    return(1);
  }

  DCB port_settings;
  memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
  port_settings.DCBlength = sizeof(port_settings);

  if(!BuildCommDCBA(baudr, &port_settings))
  {
    printf("unable to set comport dcb settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  if(!SetCommState(Cport[comport_number], &port_settings))
  {
    printf("unable to set comport cfg settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  COMMTIMEOUTS Cptimeouts;

  Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
  Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
  Cptimeouts.ReadTotalTimeoutConstant    = 0;
  Cptimeouts.WriteTotalTimeoutMultiplier = 0;
  Cptimeouts.WriteTotalTimeoutConstant   = 0;

  if(!SetCommTimeouts(Cport[comport_number], &Cptimeouts))
  {
    printf("Unable to set comport time-out settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  Cport = open(comports[comport_number], O_RDWR | O_NOCTTY | O_NDELAY);
  if(Cport <0)
  {
    perror("unable to open comport! \n\n");
    return(1);
  }

  error = tcgetattr(Cport, &old_port_settings);
  if(error !=0)
  {
    close(Cport);
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
  error = tcsetattr(Cport, TCSANOW, &new_port_settings);
  if(error !=0)
  {
    close(Cport);
    perror("unable to adjust portsettings ");
    return(1);
  }

  if(ioctl(Cport, TIOCMGET, &status) !=0)
  {
    perror("unable to get portstatus ");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
  {
    perror("unable to set portstatus ");
    return(1);
  }
#endif

  return(0);
}


int RS232_PollComport(int comport_number, unsigned char *buf, int size)
{
  int n;

#if defined(_WIN32) || defined(_WIN64)
  if(size>4096)
      size = 4096;
/* added the void pointer cast, otherwise gcc will complain about */
/* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

  ReadFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
 #ifndef __STRICT_ANSI__                      /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
  if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
 #else
  if(size>4096)
      size = 4096;
 #endif

  n = read(Cport, buf, size);
#endif
  return(n);
}

int RS232_SendByte(int comport_number, unsigned char byte)
{
  int n;
#if defined(_WIN32) || defined(_WIN64)
  WriteFile(Cport[comport_number], &byte, 1, (LPDWORD)((void *)&n), NULL);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  n = write(Cport, &byte, 1);
#endif
  if(n<0)
      return(1);
  else
      return(0);
}

int RS232_SendBuf(int comport_number, unsigned char *buf, int size)
{
#if defined(_WIN32) || defined(_WIN64)
  int n;
  if(WriteFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL))
    return(n);
  else
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  return(write(Cport, buf, size));
#endif
    return(-1);
}

void RS232_CloseComport(int comport_number)
{
#if defined(_WIN32) || defined(_WIN64)
  CloseHandle(Cport[comport_number]);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int status;

  if(ioctl(Cport, TIOCMGET, &status) !=0)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */
  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
  {
    perror("unable to set portstatus");
  }

  close(Cport);
  tcsetattr(Cport, TCSANOW, &old_port_settings);
#endif
}

/*
Constant  Description @UNIX
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
#if defined(_WIN32) || defined(_WIN64)
  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));
  if(status&MS_CTS_ON)
      return(1);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  ioctl(Cport, TIOCMGET, &status);
  if(status&TIOCM_CTS)
      return(1);
#endif
  else
      return(0);
}

int RS232_IsDSREnabled(int comport_number)
{
  int status;
#if defined(_WIN32) || defined(_WIN64)
  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));
  if(status&MS_DSR_ON)
      return(1);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  ioctl(Cport, TIOCMGET, &status);
  if(status&TIOCM_DSR)
      return(1);
#endif
  else
      return(0);
}

void RS232_enableDTR(int comport_number)
{
#if defined(_WIN32) || defined(_WIN64)
  EscapeCommFunction(Cport[comport_number], SETDTR);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int status;

  if(ioctl(Cport, TIOCMGET, &status) !=0)
    perror("unable to get portstatus");

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
    perror("unable to set portstatus");
#endif
}

void RS232_disableDTR(int comport_number)
{
#if defined(_WIN32) || defined(_WIN64)
  EscapeCommFunction(Cport[comport_number], CLRDTR);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int status;

  if(ioctl(Cport, TIOCMGET, &status) !=0)
    perror("unable to get portstatus");

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
    perror("unable to set portstatus");
#endif
}

void RS232_enableRTS(int comport_number)
{
#if defined(_WIN32) || defined(_WIN64)
  EscapeCommFunction(Cport[comport_number], SETRTS);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int status;

  if(ioctl(Cport, TIOCMGET, &status) !=0)
    perror("unable to get portstatus");

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
    perror("unable to set portstatus");
#endif
}

void RS232_disableRTS(int comport_number)
{
#if defined(_WIN32) || defined(_WIN64)
  EscapeCommFunction(Cport, CLRRTS);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__) || defined(__MACH__)
  int status;

  if(ioctl(Cport, TIOCMGET, &status) !=0)
    perror("unable to get portstatus");

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport, TIOCMSET, &status) !=0)
    perror("unable to set portstatus");
#endif
}

void RS232_cputs(int comport_number, const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte(comport_number, *(text++));
}

#ifdef __cplusplus
} /* extern "C" */
#endif
