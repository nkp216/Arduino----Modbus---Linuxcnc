/*
Reg0
2 3 4 5 6 7 8 9 10 11 12

Reg1
22 24 26 28 30 31

Reg2
32 - 47

Reg3
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

  hal_bit_t *Din00;
  hal_bit_t *Din01;
  hal_bit_t *Din02;
  hal_bit_t *Din03;
  hal_bit_t *Din04;
  hal_bit_t *Din05;
  hal_bit_t *Din06;
  hal_bit_t *Din07;
  hal_bit_t *Din08;
  hal_bit_t *Din09;
  hal_bit_t *Din010;
  hal_bit_t *Din011;
  hal_bit_t *Din012;
  hal_bit_t *Din013;
  hal_bit_t *Din014;
  hal_bit_t *Din015;  
  
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
  
  hal_bit_t *Din20;
  hal_bit_t *Din21;
  hal_bit_t *Din22;
  hal_bit_t *Din23;
  hal_bit_t *Din24;
  hal_bit_t *Din25;
  hal_bit_t *Din26;
  hal_bit_t *Din27;
  hal_bit_t *Din28;
  hal_bit_t *Din29;
  hal_bit_t *Din210;
  hal_bit_t *Din211;
  hal_bit_t *Din212;
  hal_bit_t *Din213;
  hal_bit_t *Din214;
  hal_bit_t *Din215;
  
  hal_s32_t *Din40;
  hal_s32_t *Din41;
  hal_s32_t *Din42;
  hal_s32_t *Din43;
  hal_s32_t *Din44;
  hal_s32_t *Din45;
  hal_s32_t *Din46;
  hal_s32_t *Din47;
  hal_s32_t *Din48;
  hal_s32_t *Din49;
  hal_s32_t *Din410;
  hal_s32_t *Din411;
  hal_s32_t *Din412;
  hal_s32_t *Din413;
  hal_s32_t *Din414;
  hal_s32_t *Din415;
  
  hal_bit_t *Din30;
  hal_bit_t *Din31;
  hal_bit_t *Din32;
  hal_bit_t *Din33;
  hal_bit_t *Din34;
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

static int comm_delay = 0; // JET delay counter for at-speed

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
int read_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *hal_data_block) {
    int receive_data[MAX_READ_HOLD_REGS];	
    int retval;
    int Reg0val=0;
    int Reg1val=0;
    int Reg2val=0;
    int Reg3val=0;
    //int Reg4val=0;
    int mass0[16],mass1[16],mass2[16],mass3[5];
    int x = 0;

    if (hal_data_block == NULL)
        return -1;
    if ((param==NULL) || (slavedata == NULL)) {  
        return -1;
    }
    //----------------------------------------------------------------------------------------читаем  цифровые входы --------регистр0
        retval = read_holding_registers(param, slavedata->slave, 1 , 1 , receive_data);         
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg0val  = receive_data[0] ;
        printf("Reg0val: %i\n", Reg0val); 
        x = 0;
        while ( x < 16  ){
		if ((Reg0val & 0x01) == 1 ) 
			mass0[x] = 1;
		else
			mass0[x] = 0;
		Reg0val=Reg0val>>1;
		x++; 
		}
	//*(hal_data_block->Din00)   = mass0[0];		
	//*(hal_data_block->Din01)   = mass0[1];		
	*(hal_data_block->Din02)   = mass0[2];
	*(hal_data_block->Din03)   = mass0[3];
	*(hal_data_block->Din04)   = mass0[4];
	*(hal_data_block->Din05)   = mass0[5];
	*(hal_data_block->Din06)   = mass0[6];
	*(hal_data_block->Din07)   = mass0[7];
	*(hal_data_block->Din08)   = mass0[8];
	*(hal_data_block->Din09)   = mass0[9];
	*(hal_data_block->Din010)  = mass0[10];
	*(hal_data_block->Din011)  = mass0[11];
	*(hal_data_block->Din012)  = mass0[12];
	//*(hal_data_block->Din013)  = mass0[13];
	//*(hal_data_block->Din014)  = mass0[14];
	//*(hal_data_block->Din015)  = mass0[15];                                         
        retval = 0;
        }
    }
    else {
        hal_data_block->retval = retval;
        retval = -1;
    }     
    //----------------------------------------------------------------------------------------читаем  цифровые входы --------регистр1
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
    //----------------------------------------------------------------------------------------читаем  аналоговые входы --------регистр4
    retval = read_holding_registers(param, slavedata->slave, slavedata->read_reg_start,
                                slavedata->read_reg_count, receive_data);    
    if (retval==slavedata->read_reg_count) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        *(hal_data_block->Din40)  = receive_data[0] ;
        *(hal_data_block->Din41)  = receive_data[1] ;
        *(hal_data_block->Din42)  = receive_data[2] ;
        *(hal_data_block->Din43)  = receive_data[3] ;        
        *(hal_data_block->Din44)  = receive_data[4] ;
        *(hal_data_block->Din45)  = receive_data[5] ;
        *(hal_data_block->Din46)  = receive_data[6] ;
        *(hal_data_block->Din47)  = receive_data[7] ;
        *(hal_data_block->Din48)  = receive_data[8] ;
        *(hal_data_block->Din49)  = receive_data[9] ;
        *(hal_data_block->Din410) = receive_data[10];
        *(hal_data_block->Din411) = receive_data[11];
        *(hal_data_block->Din412) = receive_data[12];
        *(hal_data_block->Din413) = receive_data[13];
        *(hal_data_block->Din414) = receive_data[14];
        *(hal_data_block->Din415) = receive_data[15];        
        retval = 0;
        }
    } else {
        hal_data_block->retval = retval;
  
        retval = -1;
    }
    //----------------------------------------------------------------------------------------читаем  цифровые входы --------регистр2       
        retval = read_holding_registers(param, slavedata->slave, 2 , 1 , receive_data);                  
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg2val  = receive_data[0] ; 
        x = 0;
        while ( x < 16  ){
		if ((Reg2val & 0x01) == 1 ) 
			mass2[x] = 1;
		else
			mass2[x] = 0;
		Reg2val=Reg2val>>1;
		x++; 
		}
	*(hal_data_block->Din20)   = mass2[0];		
	*(hal_data_block->Din21)   = mass2[1];		
	*(hal_data_block->Din22)   = mass2[2];
	*(hal_data_block->Din23)   = mass2[3];
	*(hal_data_block->Din24)   = mass2[4];
	*(hal_data_block->Din25)   = mass2[5];
	*(hal_data_block->Din26)   = mass2[6];
	*(hal_data_block->Din27)   = mass2[7];
	*(hal_data_block->Din28)   = mass2[8];
	*(hal_data_block->Din29)   = mass2[9];
	*(hal_data_block->Din210)  = mass2[10];
	*(hal_data_block->Din211)  = mass2[11];
	*(hal_data_block->Din212)  = mass2[12];
	*(hal_data_block->Din213)  = mass2[13];
	*(hal_data_block->Din214)  = mass2[14];
	*(hal_data_block->Din215)  = mass2[15];                                         
        retval = 0;
        }
    }
    else {
        hal_data_block->retval = retval;
        retval = -1;
    }
    //----------------------------------------------------------------------------------------читаем  цифровые входы --------регистр3   
        retval = read_holding_registers(param, slavedata->slave, 3 , 1 , receive_data);                  
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg3val  = receive_data[0] ;
        x=0;
                while ( x < 5  ){
		if ((Reg3val & 0x01) == 1 ) 
			mass3[x] = 1;
		else
			mass3[x] = 0;
		Reg3val=Reg3val>>1;
		x++; 
		}
	*(hal_data_block->Din30)   = mass3[0];		
	*(hal_data_block->Din31)   = mass3[1];		
	*(hal_data_block->Din32)   = mass3[2];
	*(hal_data_block->Din33)   = mass3[3];
	*(hal_data_block->Din34)   = mass3[4];
        retval = 0;
        }
    }
    else {
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

    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din00),    hal_comp_id, "%s.Din0-00" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din01),    hal_comp_id, "%s.Din0-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din02),    hal_comp_id, "%s.Din0-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din03),    hal_comp_id, "%s.Din0-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din04),    hal_comp_id, "%s.Din0-04" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din05),    hal_comp_id, "%s.Din0-05" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din06),    hal_comp_id, "%s.Din0-06" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din07),    hal_comp_id, "%s.Din0-07" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din08),    hal_comp_id, "%s.Din0-08" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din09),    hal_comp_id, "%s.Din0-09" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din010),   hal_comp_id, "%s.Din0-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din011),   hal_comp_id, "%s.Din0-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din012),   hal_comp_id, "%s.Din0-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din013),   hal_comp_id, "%s.Din0-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din014),   hal_comp_id, "%s.Din0-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din015),   hal_comp_id, "%s.Din0-15", modname); if (retval!=0) goto out_closeHAL;    
    
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din10),    hal_comp_id, "%s.Din1-00" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din11),    hal_comp_id, "%s.Din1-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din12),    hal_comp_id, "%s.Din1-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din13),    hal_comp_id, "%s.Din1-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din14),    hal_comp_id, "%s.Din1-04" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din15),    hal_comp_id, "%s.Din1-05" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din16),    hal_comp_id, "%s.Din1-06" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din17),    hal_comp_id, "%s.Din1-07" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din18),    hal_comp_id, "%s.Din1-08" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din19),    hal_comp_id, "%s.Din1-09" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din110),   hal_comp_id, "%s.Din1-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din111),   hal_comp_id, "%s.Din1-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din112),   hal_comp_id, "%s.Din1-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din113),   hal_comp_id, "%s.Din1-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din114),   hal_comp_id, "%s.Din1-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din115),   hal_comp_id, "%s.Din1-15", modname); if (retval!=0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din20),    hal_comp_id, "%s.Din2-00" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din21),    hal_comp_id, "%s.Din2-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din22),    hal_comp_id, "%s.Din2-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din23),    hal_comp_id, "%s.Din2-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din24),    hal_comp_id, "%s.Din2-04" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din25),    hal_comp_id, "%s.Din2-05" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din26),    hal_comp_id, "%s.Din2-06" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din27),    hal_comp_id, "%s.Din2-07" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din28),    hal_comp_id, "%s.Din2-08" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din29),    hal_comp_id, "%s.Din2-09" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din210),   hal_comp_id, "%s.Din2-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din211),   hal_comp_id, "%s.Din2-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din212),   hal_comp_id, "%s.Din2-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din213),   hal_comp_id, "%s.Din2-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din214),   hal_comp_id, "%s.Din2-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din215),   hal_comp_id, "%s.Din2-15", modname); if (retval!=0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din30),    hal_comp_id, "%s.Din3-00" , modname); if (retval!=0) goto out_closeHAL;    
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din31),    hal_comp_id, "%s.Din3-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din32),    hal_comp_id, "%s.Din3-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din33),    hal_comp_id, "%s.Din3-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->Din34),    hal_comp_id, "%s.Din3-04" , modname); if (retval!=0) goto out_closeHAL;
    
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din40),    hal_comp_id, "%s.Din4-00" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din41),    hal_comp_id, "%s.Din4-01" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din42),    hal_comp_id, "%s.Din4-02" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din43),    hal_comp_id, "%s.Din4-03" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din44),    hal_comp_id, "%s.Din4-04" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din45),    hal_comp_id, "%s.Din4-05" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din46),    hal_comp_id, "%s.Din4-06" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din47),    hal_comp_id, "%s.Din4-07" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din48),    hal_comp_id, "%s.Din4-08" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din49),    hal_comp_id, "%s.Din4-09" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din410),   hal_comp_id, "%s.Din4-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din411),   hal_comp_id, "%s.Din4-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din412),   hal_comp_id, "%s.Din4-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din413),   hal_comp_id, "%s.Din4-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din414),   hal_comp_id, "%s.Din4-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Din415),   hal_comp_id, "%s.Din4-15", modname); if (retval!=0) goto out_closeHAL;
    
    
    *(haldata->Din00)  = 0;    
    *(haldata->Din01)  = 0; 
    *(haldata->Din02)  = 0;
    *(haldata->Din03)  = 0;
    *(haldata->Din04)  = 0;
    *(haldata->Din05)  = 0;
    *(haldata->Din06)  = 0;
    *(haldata->Din07)  = 0;
    *(haldata->Din08)  = 0;
    *(haldata->Din09)  = 0;
    *(haldata->Din010) = 0;
    *(haldata->Din011) = 0;
    *(haldata->Din012) = 0;
    *(haldata->Din013) = 0;
    *(haldata->Din014) = 0;
    *(haldata->Din015) = 0;    
 
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
  
    *(haldata->Din20)  = 0;    
    *(haldata->Din21)  = 0; 
    *(haldata->Din22)  = 0;
    *(haldata->Din23)  = 0;
    *(haldata->Din24)  = 0;
    *(haldata->Din25)  = 0;
    *(haldata->Din26)  = 0;
    *(haldata->Din27)  = 0;
    *(haldata->Din28)  = 0;
    *(haldata->Din29)  = 0;
    *(haldata->Din210) = 0;
    *(haldata->Din211) = 0;
    *(haldata->Din212) = 0;
    *(haldata->Din213) = 0;
    *(haldata->Din214) = 0;
    *(haldata->Din215) = 0;

    *(haldata->Din30) = 0;    
    *(haldata->Din31) = 0; 
    *(haldata->Din32) = 0;
    *(haldata->Din33) = 0;
    *(haldata->Din34) = 0;
    
    *(haldata->Din40)  = 0;    
    *(haldata->Din41)  = 0; 
    *(haldata->Din42)  = 0;
    *(haldata->Din43)  = 0;
    *(haldata->Din44)  = 0;
    *(haldata->Din45)  = 0;
    *(haldata->Din46)  = 0;
    *(haldata->Din47)  = 0;
    *(haldata->Din48)  = 0;
    *(haldata->Din49)  = 0;
    *(haldata->Din410) = 0;
    *(haldata->Din411) = 0;
    *(haldata->Din412) = 0;
    *(haldata->Din413) = 0;
    *(haldata->Din414) = 0;
    *(haldata->Din415) = 0;

    while (done==0) {
        read_data(&mb_param, &slavedata, haldata);
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
