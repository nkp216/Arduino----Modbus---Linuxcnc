
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
  hal_s32_t *Ain0; hal_s32_t *Ain1; hal_s32_t *Ain2; hal_s32_t *Ain3;  hal_s32_t *Ain4;  hal_s32_t *Ain5;  hal_s32_t *Ain6;  hal_s32_t *Ain7;
  hal_s32_t *Ain8; hal_s32_t *Ain9; hal_s32_t *Ain10;hal_s32_t *Ain11; hal_s32_t *Ain12; hal_s32_t *Ain13; hal_s32_t *Ain14; hal_s32_t *Ain15;      
  hal_float_t *pwm1; hal_float_t *pwm2; hal_float_t *pwm3; hal_float_t *pwm4; hal_float_t *pwm5; hal_float_t *pwm6; hal_bit_t *Dout1;
  hal_bit_t *Dout2; hal_bit_t *Dout3; hal_bit_t *Dout4; hal_bit_t *Dout8; hal_bit_t *Dout9; hal_bit_t *Dout12; hal_s32_t retval;
  hal_float_t looptime; hal_bit_t *Din1; hal_bit_t *Din2; hal_bit_t *Din3; hal_bit_t *Din4; hal_bit_t *Din5; hal_bit_t *Din6;
  hal_bit_t *Din7; hal_bit_t *Din8; hal_bit_t *Din9; hal_bit_t *Din10; hal_bit_t *Din11; hal_bit_t *Din12; hal_bit_t *Din13; hal_bit_t *Din14;
  hal_bit_t *Din15; hal_bit_t *Din16;
} haldata_t;
static  int regarg[4];
static int done;
char *modname = "arduino";
static void quit(int sig) {
    done = 1;
}
int write_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *haldata) {  //==============  write  ====================
     int retval;
     int  v0;
     v0 = (*(haldata->Dout1)) | (*(haldata->Dout2))<<1| (*(haldata->Dout3))<<2| (*(haldata->Dout4)<<4)| (*(haldata->Dout8))<<7|     
      (*(haldata->Dout9))<<8| (*(haldata->Dout12)<<11); 

     //printf("gs2_vfd: pwm1: %i\n", v0);       
     retval = preset_single_register(param, 1, 0, v0); //посылаем в устройство1, регистр 0 число v0
     preset_single_register(param, 1, 2, v0);
     //шим 
     preset_single_register(param, 1, 13, abs((int)(*(haldata->pwm1))));   //(13,15,16,19,20,21)
     preset_single_register(param, 1, 15, abs((int)(*(haldata->pwm2))));   
     preset_single_register(param, 1, 16, abs((int)(*(haldata->pwm3))));
     preset_single_register(param, 1, 19, abs((int)(*(haldata->pwm4))));
     preset_single_register(param, 1, 20, abs((int)(*(haldata->pwm5))));
     preset_single_register(param, 1, 21, abs((int)(*(haldata->pwm6))));
     
 //printf("regarg[1]: %i\n",regarg[1]);
 
    return retval;
}
int read_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *hal_data_block) {
    int receive_data[MAX_READ_HOLD_REGS];	
    int retval;
    int Reg1val;
    int Reg2val;
    int mass[16];
    int x = 0;

    if (hal_data_block == NULL)
        return -1;
    if ((param==NULL) || (slavedata == NULL)) {  
        return -1;
    }
    retval = read_holding_registers(param, slavedata->slave, slavedata->read_reg_start,
                                slavedata->read_reg_count, receive_data);    
    if (retval==slavedata->read_reg_count) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        *(hal_data_block->Ain0)  = receive_data[0] ;
        *(hal_data_block->Ain1)  = receive_data[1] ;
        *(hal_data_block->Ain2)  = receive_data[2] ;
        *(hal_data_block->Ain3)  = receive_data[3] ;	
        *(hal_data_block->Ain4)  = receive_data[4] ;
        *(hal_data_block->Ain5)  = receive_data[5] ;
        *(hal_data_block->Ain6)  = receive_data[6] ;
        *(hal_data_block->Ain7)  = receive_data[7] ;
        *(hal_data_block->Ain8)  = receive_data[8] ;
        *(hal_data_block->Ain9)  = receive_data[9] ;
        *(hal_data_block->Ain10) = receive_data[10];
        *(hal_data_block->Ain11) = receive_data[11];
        *(hal_data_block->Ain12) = receive_data[12];
        *(hal_data_block->Ain13) = receive_data[13];
        *(hal_data_block->Ain14) = receive_data[14];
        *(hal_data_block->Ain15) = receive_data[15];        
        retval = 0;
        }
    } else {
        hal_data_block->retval = retval;
  
        retval = -1;
    }
    //----------------------------------------------------------------------------------------читаем  цифровые входы
        retval = read_holding_registers(param, slavedata->slave, 2 ,   2 , receive_data);
                  
        if (retval==2) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg1val  = receive_data[0] ; //читаем регистр
        Reg2val  = receive_data[1] ;
        //========================================
        while ( x < 16  ){
		if ((Reg1val & 0x01) == 1 ) 
			mass[x] = 1;
		else
			mass[x] = 0;
		Reg1val=Reg1val>>1;
		x++; 
		}
	*(hal_data_block->Din1)   = mass[0];		
	*(hal_data_block->Din2)   = mass[1];
	*(hal_data_block->Din3)   = mass[2];
	*(hal_data_block->Din4)   = mass[3];
	*(hal_data_block->Din5)   = mass[4];
	*(hal_data_block->Din6)   = mass[5];
	*(hal_data_block->Din7)   = mass[6];
	*(hal_data_block->Din8)   = mass[7];
	*(hal_data_block->Din9)   = mass[8];
	*(hal_data_block->Din10)  = mass[9];
	*(hal_data_block->Din11)  = mass[10];
	*(hal_data_block->Din12)  = mass[11];
	*(hal_data_block->Din13)  = mass[12];
	*(hal_data_block->Din14)  = mass[13];
	*(hal_data_block->Din15)  = mass[14];
	*(hal_data_block->Din16)  = mass[15];

	//=======================================                                          
        retval = 0;
        }
    } else {
        hal_data_block->retval = retval;
        retval = -1;
    }                                                          
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
    char *device, *parity ;
    char *regset ;
    int i , rt;
    
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
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain0), hal_comp_id, "%s.Ain0", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain1), hal_comp_id, "%s.Ain1", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain2), hal_comp_id, "%s.Ain2", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain3), hal_comp_id, "%s.Ain3", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain4), hal_comp_id, "%s.Ain4", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain5), hal_comp_id, "%s.Ain5", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain6), hal_comp_id, "%s.Ain6", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain7), hal_comp_id, "%s.Ain7", modname);
    if (retval!=0) goto out_closeHAL;        
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain8), hal_comp_id, "%s.Ain8", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain9), hal_comp_id, "%s.Ain9", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain10), hal_comp_id, "%s.Ain10", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain11), hal_comp_id, "%s.Ain11", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain12), hal_comp_id, "%s.Ain12", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain13), hal_comp_id, "%s.Ain13", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain14), hal_comp_id, "%s.Ain14", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain15), hal_comp_id, "%s.Ain15", modname);
    if (retval!=0) goto out_closeHAL; 
    
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm1), hal_comp_id, "%s.pwm1", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm2), hal_comp_id, "%s.pwm2", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm3), hal_comp_id, "%s.pwm3", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm4), hal_comp_id, "%s.pwm4", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm5), hal_comp_id, "%s.pwm5", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm6), hal_comp_id, "%s.pwm6", modname);
    if (retval!=0) goto out_closeHAL;
        
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout1),   hal_comp_id, "%s.Dout1", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout2),   hal_comp_id, "%s.Dout2", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout3),   hal_comp_id, "%s.Dout3", modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout4),   hal_comp_id, "%s.Dout4", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout8),   hal_comp_id, "%s.Dout8", modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout9),   hal_comp_id, "%s.Dout9", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout12),  hal_comp_id, "%s.Dout12", modname);
    if (retval!=0) goto out_closeHAL;
    
    
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din1),    hal_comp_id, "%s.Din1" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din2),    hal_comp_id, "%s.Din2" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din3),    hal_comp_id, "%s.Din3" , modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din4),    hal_comp_id, "%s.Din4" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din5),    hal_comp_id, "%s.Din5" , modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din6),    hal_comp_id, "%s.Din6" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din7),    hal_comp_id, "%s.Din7" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din8),    hal_comp_id, "%s.Din8" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din9),    hal_comp_id, "%s.Din9" , modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din10),   hal_comp_id, "%s.Din10", modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din11),   hal_comp_id, "%s.Din11", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din12),   hal_comp_id, "%s.Din12", modname); 
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din13),   hal_comp_id, "%s.Din13", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din14),   hal_comp_id, "%s.Din14", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din15),   hal_comp_id, "%s.Din15", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din16),   hal_comp_id, "%s.Din16", modname);
    if (retval!=0) goto out_closeHAL;    
    
    *(haldata->Ain0)  = 0;
    *(haldata->Ain1)  = 0;
    *(haldata->Ain2)  = 0;
    *(haldata->Ain3)  = 0;
    *(haldata->Ain4)  = 0;
    *(haldata->Ain5)  = 0;
    *(haldata->Ain6)  = 0;
    *(haldata->Ain7)  = 0;
    *(haldata->Ain8)  = 0;
    *(haldata->Ain9)  = 0;
    *(haldata->Ain10) = 0;
    *(haldata->Ain11) = 0; 
    *(haldata->Ain12) = 0;
    *(haldata->Ain13) = 0;
    *(haldata->Ain14) = 0;
    *(haldata->Ain15) = 0;

    *(haldata->Dout1) = 0;
    *(haldata->Dout2) = 0;
    *(haldata->Dout3) = 0;
    *(haldata->Dout4) = 0;
    *(haldata->Dout8) = 0;
    *(haldata->Dout9) = 0;
    *(haldata->Dout12) = 0;
    
    *(haldata->Din1) = 0; 
    *(haldata->Din2) = 0;
    *(haldata->Din3) = 0;
    *(haldata->Din4) = 0;
    *(haldata->Din5) = 0;
    *(haldata->Din6) = 0;
    *(haldata->Din7) = 0;
    *(haldata->Din8) = 0;
    *(haldata->Din9) = 0;
    *(haldata->Din10) = 0;
    *(haldata->Din11) = 0;
    *(haldata->Din12) = 0;
    *(haldata->Din13) = 0;
    *(haldata->Din14) = 0;
    *(haldata->Din15) = 0;
    *(haldata->Din16) = 0;
  
    getopt(argc,argv,"v:");
	regset = optarg; 	
    rt = atoi(regset);
    for(i = 0;i < 4;i++){
	regarg[i] = rt & 0x01;
	rt = rt >> 1;
	}
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
