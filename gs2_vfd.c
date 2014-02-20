/*
PINS_PWM
3 5 6 9 10 11 

PINS_IN
16 - 31

PINS_OUT
32 - 47
48 49 50 51 52
*/
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include "rtapi.h"
#include "hal.h"
#include "modbus.h"

#define START_REGISTER_R	0x1E
#define NUM_REGISTERS_R		16
#define START_REGISTER_W	0
#define NUM_REGISTERS_W		3
#undef DEBUG

typedef struct {
	int slave;		
	int read_reg_start;	
	int read_reg_count;	
	int write_reg_start;	
	int write_reg_count;	
} slavedata_t;

typedef struct {
  hal_s32_t retval;
  hal_float_t looptime; 
   
  hal_s32_t *pwm03;
  hal_s32_t *pwm05;
  hal_s32_t *pwm06;
  hal_s32_t *pwm09;
  hal_s32_t *pwm10;
  hal_s32_t *pwm11;  
  
  hal_bit_t *Din10;
  hal_bit_t *Din11;
  hal_bit_t *Din12;
  hal_bit_t *Din13;
  hal_bit_t *Din14;
  hal_bit_t *Din15;
  hal_bit_t *Din16;
  hal_bit_t *Din17;
  hal_bit_t *Din18;
  hal_bit_t *Din19;
  hal_bit_t *Din110;
  hal_bit_t *Din111;
  hal_bit_t *Din112;
  hal_bit_t *Din113;
  hal_bit_t *Din114;
  hal_bit_t *Din115;  
  
  hal_bit_t *Dout20;
  hal_bit_t *Dout21;
  hal_bit_t *Dout22;
  hal_bit_t *Dout23;
  hal_bit_t *Dout24;
  hal_bit_t *Dout25;
  hal_bit_t *Dout26;
  hal_bit_t *Dout27;
  hal_bit_t *Dout28;
  hal_bit_t *Dout29;
  hal_bit_t *Dout210;
  hal_bit_t *Dout211;
  hal_bit_t *Dout212;
  hal_bit_t *Dout213;
  hal_bit_t *Dout214;
  hal_bit_t *Dout215;
 
  hal_bit_t *Dout30;
  hal_bit_t *Dout31;
  hal_bit_t *Dout32;
  hal_bit_t *Dout33;
  hal_bit_t *Dout34; 
     
  hal_s32_t *Ain40;
  hal_s32_t *Ain41;
  hal_s32_t *Ain42;
  hal_s32_t *Ain43;
  hal_s32_t *Ain44;
  hal_s32_t *Ain45;
  hal_s32_t *Ain46;
  hal_s32_t *Ain47;
  hal_s32_t *Ain48;
  hal_s32_t *Ain49;
  hal_s32_t *Ain410;
  hal_s32_t *Ain411;
  hal_s32_t *Ain412;
  hal_s32_t *Ain413;
  hal_s32_t *Ain414;
  hal_s32_t *Ain415;
} haldata_t;
static int done;
char *modname = "arduino";
static struct option long_options[] = {
    {"bits", 1, 0, 'b'},
    {"device", 1, 0, 'd'},
    {"debug", 0, 0, 'g'},
    {"help", 0, 0, 'h'},
    {"name", 1, 0, 'n'},
    {"parity", 1, 0, 'p'},
    {"rate", 1, 0, 'r'},
    {"stopbits", 1, 0, 's'},
    {"target", 1, 0, 't'},
    {"verbose", 0, 0, 'v'},
    {0,0,0,0}
};
static char *option_string = "b:d:hn:p:r:s:t:v";
static char *bitstrings[] = {"5", "6", "7", "8", NULL};
static char *paritystrings[] = {"even", "odd", "none", NULL};
static char *ratestrings[] = {"110", "300", "600", "1200", "2400", "4800", "9600",
    "19200", "38400", "57600", "115200", NULL};
static char *stopstrings[] = {"1", "2", NULL};

static void quit(int sig) {
    done = 1;
}
int match_string(char *string, char **matches) {
    int len, which, match;
    which=0;
    match=-1;
    if ((matches==NULL) || (string==NULL)) return -1;
    len = strlen(string);
    while (matches[which] != NULL) {
        if ((!strncmp(string, matches[which], len)) && (len <= strlen(matches[which]))) {
            if (match>=0) return -1;        // multiple matches
            match=which;
        }
        ++which;
    }
    return match;
}
void usage(int argc, char **argv) {
    printf("Usage:  %s [options]\n", argv[0]);
    printf(
    "This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
    "    loadusr gs2_vfd\n"
    "There are several command-line options.  Options that have a set list of possible values may\n"
    "    be set by using any number of characters that are unique.  For example, --rate 5 will use\n"
    "    a baud rate of 57600, since no other available baud rates start with \"5\"\n"
    "-b or --bits <n> (default 8)\n"
    "    Set number of data bits to <n>, where n must be from 5 to 8 inclusive\n"
    "-d or --device <path> (default /dev/ttyS0)\n"
    "    Set the name of the serial device node to use\n"
    "-g or --debug\n"
    "    Turn on debugging messages.  This will also set the verbose flag.  Debug mode will cause\n"
    "    all modbus messages to be printed in hex on the terminal.\n"
    "-n or --name <string> (default gs2_vfd)\n"
    "    Set the name of the HAL module.  The HAL comp name will be set to <string>, and all pin\n"
    "    and parameter names will begin with <string>.\n"
    "-p or --parity {even,odd,none} (defalt odd)\n"
    "    Set serial parity to even, odd, or none.\n"
    "-r or --rate <n> (default 38400)\n"
    "    Set baud rate to <n>.  It is an error if the rate is not one of the following:\n"
    "    110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200\n"
    "-s or --stopbits {1,2} (default 1)\n"
    "    Set serial stop bits to 1 or 2\n"
    "-t or --target <n> (default 1)\n"
    "    Set MODBUS target (slave) number.  This must match the device number you set on the GS2.\n"
    "-v or --verbose\n"
    "    Turn on debug messages.  Note that if there are serial errors, this may become annoying.\n"
    "    At the moment, it doesn't make much difference most of the time.\n");
}
int write_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *haldata) {  //==============  write  ====================
     int retval ;
     int  v2 ,v3;
     //шим (пины:3,5,6,9,10,11)
     preset_single_register(param, 1, 13, abs((int)(*(haldata->pwm03))));   //(Reg:13,15,16,19,20,21)
     preset_single_register(param, 1, 15, abs((int)(*(haldata->pwm05))));   
     preset_single_register(param, 1, 16, abs((int)(*(haldata->pwm06))));
     preset_single_register(param, 1, 19, abs((int)(*(haldata->pwm09))));
     preset_single_register(param, 1, 20, abs((int)(*(haldata->pwm10))));
     preset_single_register(param, 1, 21, abs((int)(*(haldata->pwm11))));
     
    
     v2 = (*(haldata->Dout20))      | (*(haldata->Dout21))<<1   | (*(haldata->Dout22))<<2   | (*(haldata->Dout23))<<3   | 
          (*(haldata->Dout24))<<4   | (*(haldata->Dout25))<<5   | (*(haldata->Dout26))<<6   | (*(haldata->Dout27))<<7   | 
          (*(haldata->Dout28))<<8   | (*(haldata->Dout29))<<9   | (*(haldata->Dout210))<<10 | (*(haldata->Dout211))<<11 | 
          (*(haldata->Dout212))<<12 | (*(haldata->Dout213))<<13 | (*(haldata->Dout214))<<14 | (*(haldata->Dout215))<<15 ;
     retval = preset_single_register(param, 1, 2, v2); //посылаем в устройство1, регистр 0 число v2

     v3 = (*(haldata->Dout30)) | (*(haldata->Dout31))<<1 | (*(haldata->Dout32))<<2 | (*(haldata->Dout33)<<3) | (*(haldata->Dout34))<<4 ;
     retval = preset_single_register(param, 1, 3, v3); //посылаем в устройство1, регистр 3 число v3
     
    return retval;
}
int read_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *hal_data_block) {
    int receive_data[MAX_READ_HOLD_REGS];	
    int retval;
    int Reg1val=0;
    int mass1[16];
    int x = 0;

    if (hal_data_block == NULL)
        return -1;
    if ((param==NULL) || (slavedata == NULL)) {  
        return -1;
    }  
    //---------------------------------------читаем  цифровые входы --------регистр1
        retval = read_holding_registers(param, slavedata->slave, 1 , 1 , receive_data);         
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg1val  = receive_data[0] ;
        x = 0;
        while ( x < 16  ){
		if ((Reg1val & 0x01) == 1 ) 
			mass1[x] = 1;
		else
			mass1[x] = 0;
		Reg1val=Reg1val>>1;
		x++; 
		}
	*(hal_data_block->Din10)   = mass1[0];		
	*(hal_data_block->Din11)   = mass1[1];		
	*(hal_data_block->Din12)   = mass1[2];
	*(hal_data_block->Din13)   = mass1[3];
	*(hal_data_block->Din14)   = mass1[4];
	*(hal_data_block->Din15)   = mass1[5];
	*(hal_data_block->Din16)   = mass1[6];
	*(hal_data_block->Din17)   = mass1[7];
	*(hal_data_block->Din18)   = mass1[8];
	*(hal_data_block->Din19)   = mass1[9];
	*(hal_data_block->Din110)  = mass1[10];
	*(hal_data_block->Din111)  = mass1[11];
	*(hal_data_block->Din112)  = mass1[12];
	*(hal_data_block->Din113)  = mass1[13];
	*(hal_data_block->Din114)  = mass1[14];
	*(hal_data_block->Din115)  = mass1[15];                                         
        retval = 0;
        }
    }
    else {
        hal_data_block->retval = retval;
        retval = -1;
    }    
    //----------------------------------------------------читаем  аналоговые входы --------регистр4
    retval = read_holding_registers(param, slavedata->slave, slavedata->read_reg_start,
                                slavedata->read_reg_count, receive_data);    
    if (retval==slavedata->read_reg_count) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        *(hal_data_block->Ain40)  = receive_data[0] ;
        *(hal_data_block->Ain41)  = receive_data[1] ;
        *(hal_data_block->Ain42)  = receive_data[2] ;
        *(hal_data_block->Ain43)  = receive_data[3] ;        
        *(hal_data_block->Ain44)  = receive_data[4] ;
        *(hal_data_block->Ain45)  = receive_data[5] ;
        *(hal_data_block->Ain46)  = receive_data[6] ;
        *(hal_data_block->Ain47)  = receive_data[7] ;
        *(hal_data_block->Ain48)  = receive_data[8] ;
        *(hal_data_block->Ain49)  = receive_data[9] ;
        *(hal_data_block->Ain410) = receive_data[10];
        *(hal_data_block->Ain411) = receive_data[11];
        *(hal_data_block->Ain412) = receive_data[12];
        *(hal_data_block->Ain413) = receive_data[13];
        *(hal_data_block->Ain414) = receive_data[14];
        *(hal_data_block->Ain415) = receive_data[15];        
        retval = 0;
        }
    } else {
        hal_data_block->retval = retval;
  
        retval = -1;
    }
//========================================                                                           
    return retval;
}
int main(int argc, char **argv)
{
    int retval;
    modbus_param_t mb_param;
    haldata_t *haldata;
    slavedata_t slavedata;
    int hal_comp_id;
    struct timespec loop_timespec, remaining;
    int baud, bits, stopbits, debug, verbose;
    char *device, *parity, *endarg;
    int opt;
    int argindex, argvalue;
    
    done = 0;
    baud = 19200;
    bits = 8;
    stopbits = 1;
    debug = 0;
    verbose = 0;
    device = "/dev/ttyACM0";
    parity = "none";

    slavedata.slave = 1;
    slavedata.read_reg_start = START_REGISTER_R;
    slavedata.read_reg_count = NUM_REGISTERS_R;
    slavedata.write_reg_start = START_REGISTER_W;
    slavedata.write_reg_count = NUM_REGISTERS_R;
    
        // process command line options
    while ((opt=getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
            case 'b':   // serial data bits, probably should be 8 (and defaults to 8)
                argindex=match_string(optarg, bitstrings);
                if (argindex<0) {
                
                    printf("gs2_vfd: ERROR: invalid number of bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                bits = atoi(bitstrings[argindex]);
                break;
            case 'd':   // device name, default /dev/ttyS0
                // could check the device name here, but we'll leave it to the library open
                if (strlen(optarg) > FILENAME_MAX) {
                    printf("gs2_vfd: ERROR: device node name is too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                device = strdup(optarg);
                break;
            case 'g':
                debug = 1;
                verbose = 1;
                break;
            case 'n':   // module base name
                if (strlen(optarg) > HAL_NAME_LEN-20) {
                    printf("gs2_vfd: ERROR: HAL module name too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                modname = strdup(optarg);
                break;
            case 'p':   // parity, should be a string like "even", "odd", or "none"
                argindex=match_string(optarg, paritystrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid parity: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                parity = paritystrings[argindex];
                break;
            case 'r':   // Baud rate, 38400 default
                argindex=match_string(optarg, ratestrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid baud rate: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                baud = atoi(ratestrings[argindex]);
                break;
            case 's':   // stop bits, defaults to 1
                argindex=match_string(optarg, stopstrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid number of stop bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                stopbits = atoi(stopstrings[argindex]);
                break;
            case 't':   // target number (MODBUS ID), default 1
                argvalue = strtol(optarg, &endarg, 10);
                if ((*endarg != '\0') || (argvalue < 1) || (argvalue > 254)) {
                    printf("gs2_vfd: ERROR: invalid slave number: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                slavedata.slave = argvalue;
                break;
            case 'v':   // verbose mode (print modbus errors and other information), default 0
                verbose = 1;
                break;
            case 'h':
            default:
                usage(argc, argv);
                exit(0);
                break;
        }
    }
    printf("%s: device='%s', baud=%d, bits=%d, parity='%s', stopbits=%d, address=%d, verbose=%d\n",
           modname, device, baud, bits, parity, stopbits, slavedata.slave, debug);
    /* point TERM and INT signals at our quit function */
    /* if a signal is received between here and the main loop, it should prevent
            some initialization from happening */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    
    modbus_init_rtu(&mb_param, device, baud, parity, bits, stopbits, verbose);

    mb_param.debug = debug;
    if (((retval = modbus_connect(&mb_param))!=0) || done) {
        printf("%s: ERROR: couldn't open serial device\n", modname);
        goto out_noclose;
    }

    hal_comp_id = hal_init(modname);
    if ((hal_comp_id < 0) || done) {
        printf("%s: ERROR: hal_init failed\n", modname);
        retval = hal_comp_id;
        goto out_close;
    }
    haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if ((haldata == 0) || done) {
        printf("%s: ERROR: unable to allocate shared memory\n", modname);
        retval = -1;
        goto out_close;
    }
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm03), hal_comp_id, "%s.pwm03", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm05), hal_comp_id, "%s.pwm05", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm06), hal_comp_id, "%s.pwm06", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm09), hal_comp_id, "%s.pwm09", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm10), hal_comp_id, "%s.pwm10", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_IN, &(haldata->pwm11), hal_comp_id, "%s.pwm11", modname); if (retval!=0) goto out_closeHAL;  
    
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din10),    hal_comp_id, "%s.Din1-16" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din11),    hal_comp_id, "%s.Din1-17" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din12),    hal_comp_id, "%s.Din1-18" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din13),    hal_comp_id, "%s.Din1-19" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din14),    hal_comp_id, "%s.Din1-20" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din15),    hal_comp_id, "%s.Din1-21" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din16),    hal_comp_id, "%s.Din1-22" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din17),    hal_comp_id, "%s.Din1-23" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din18),    hal_comp_id, "%s.Din1-24" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din19),    hal_comp_id, "%s.Din1-25" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din110),   hal_comp_id, "%s.Din1-26", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din111),   hal_comp_id, "%s.Din1-27", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din112),   hal_comp_id, "%s.Din1-28", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din113),   hal_comp_id, "%s.Din1-29", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din114),   hal_comp_id, "%s.Din1-30", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din115),   hal_comp_id, "%s.Din1-31", modname); if (retval!=0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout20),    hal_comp_id, "%s.Dout2-32" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout21),    hal_comp_id, "%s.Dout2-33" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout22),    hal_comp_id, "%s.Dout2-34" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout23),    hal_comp_id, "%s.Dout2-35" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout24),    hal_comp_id, "%s.Dout2-36" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout25),    hal_comp_id, "%s.Dout2-37" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout26),    hal_comp_id, "%s.Dout2-38" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout27),    hal_comp_id, "%s.Dout2-39" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout28),    hal_comp_id, "%s.Dout2-40" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout29),    hal_comp_id, "%s.Dout2-41" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout210),   hal_comp_id, "%s.Dout2-42", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout211),   hal_comp_id, "%s.Dout2-43", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout212),   hal_comp_id, "%s.Dout2-44", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout213),   hal_comp_id, "%s.Dout2-45", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout214),   hal_comp_id, "%s.Dout2-46", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout215),   hal_comp_id, "%s.Dout2-47", modname); if (retval!=0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout30),    hal_comp_id, "%s.Dout3-48" , modname); if (retval!=0) goto out_closeHAL;    
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout31),    hal_comp_id, "%s.Dout3-49" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout32),    hal_comp_id, "%s.Dout3-50" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout33),    hal_comp_id, "%s.Dout3-51" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout34),    hal_comp_id, "%s.Dout3-52" , modname); if (retval!=0) goto out_closeHAL;
    
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain40),    hal_comp_id, "%s.Ain4-00" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain41),    hal_comp_id, "%s.Ain4-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain42),    hal_comp_id, "%s.Ain4-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain43),    hal_comp_id, "%s.Ain4-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain44),    hal_comp_id, "%s.Ain4-04" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain45),    hal_comp_id, "%s.Ain4-05" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain46),    hal_comp_id, "%s.Ain4-06" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain47),    hal_comp_id, "%s.Ain4-07" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain48),    hal_comp_id, "%s.Ain4-08" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain49),    hal_comp_id, "%s.Ain4-09" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain410),   hal_comp_id, "%s.Ain4-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain411),   hal_comp_id, "%s.Ain4-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain412),   hal_comp_id, "%s.Ain4-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain413),   hal_comp_id, "%s.Ain4-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain414),   hal_comp_id, "%s.Ain4-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain415),   hal_comp_id, "%s.Ain4-15", modname); if (retval!=0) goto out_closeHAL;       
 
    *(haldata->Din10)  = 0;    
    *(haldata->Din11)  = 0; 
    *(haldata->Din12)  = 0;
    *(haldata->Din13)  = 0;
    *(haldata->Din14)  = 0;
    *(haldata->Din15)  = 0;
    *(haldata->Din16)  = 0;
    *(haldata->Din17)  = 0;
    *(haldata->Din18)  = 0;
    *(haldata->Din19)  = 0;
    *(haldata->Din110) = 0;
    *(haldata->Din111) = 0;
    *(haldata->Din112) = 0;
    *(haldata->Din113) = 0;
    *(haldata->Din114) = 0;
    *(haldata->Din115) = 0;
  
    *(haldata->Dout20)  = 0;    
    *(haldata->Dout21)  = 0; 
    *(haldata->Dout22)  = 0;
    *(haldata->Dout23)  = 0;
    *(haldata->Dout24)  = 0;
    *(haldata->Dout25)  = 0;
    *(haldata->Dout26)  = 0;
    *(haldata->Dout27)  = 0;
    *(haldata->Dout28)  = 0;
    *(haldata->Dout29)  = 0;
    *(haldata->Dout210) = 0;
    *(haldata->Dout211) = 0;
    *(haldata->Dout212) = 0;
    *(haldata->Dout213) = 0;
    *(haldata->Dout214) = 0;
    *(haldata->Dout215) = 0;

    *(haldata->Dout30) = 0;    
    *(haldata->Dout31) = 0; 
    *(haldata->Dout32) = 0;
    *(haldata->Dout33) = 0;
    *(haldata->Dout34) = 0;
    
    *(haldata->Ain40)  = 0;    
    *(haldata->Ain41)  = 0; 
    *(haldata->Ain42)  = 0;
    *(haldata->Ain43)  = 0;
    *(haldata->Ain44)  = 0;
    *(haldata->Ain45)  = 0;
    *(haldata->Ain46)  = 0;
    *(haldata->Ain47)  = 0;
    *(haldata->Ain48)  = 0;
    *(haldata->Ain49)  = 0;
    *(haldata->Ain410) = 0;
    *(haldata->Ain411) = 0;
    *(haldata->Ain412) = 0;
    *(haldata->Ain413) = 0;
    *(haldata->Ain414) = 0;
    *(haldata->Ain415) = 0;

    while (done==0) {
        read_data(&mb_param, &slavedata, haldata);
        write_data(&mb_param, &slavedata, haldata);
        if (haldata->looptime < 0.001) haldata->looptime = 0.001;
        if (haldata->looptime > 2.0) haldata->looptime = 2.0;
        loop_timespec.tv_sec = (time_t)(haldata->looptime);
        loop_timespec.tv_nsec = (long)((haldata->looptime - loop_timespec.tv_sec) * 1000000000l);
        nanosleep(&loop_timespec, &remaining);
    }    
    retval = 0;
out_closeHAL:
    hal_exit(hal_comp_id);
out_close:
    modbus_close(&mb_param);
out_noclose:
    return retval;
}
