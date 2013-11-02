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
  hal_s32_t *Ain0; hal_s32_t *Ain1; hal_s32_t *Ain2; hal_s32_t *Ain3;  hal_s32_t *Ain4;  hal_s32_t *Ain5;  hal_s32_t *Ain6;  hal_s32_t *Ain7;
  hal_s32_t *Ain8; hal_s32_t *Ain9; hal_s32_t *Ain10;hal_s32_t *Ain11; hal_s32_t *Ain12; hal_s32_t *Ain13; hal_s32_t *Ain14; hal_s32_t *Ain15;      
  hal_float_t *pwm1; hal_float_t *pwm2; hal_float_t *pwm3; hal_float_t *pwm4; hal_float_t *pwm5; hal_float_t *pwm6; hal_bit_t *Dout1;
  hal_bit_t *Dout2; hal_bit_t *Dout3; hal_bit_t *Dout4; hal_bit_t *Dout8; hal_bit_t *Dout9; hal_bit_t *Dout12; hal_bit_t *Dout30;
  hal_bit_t *Dout31;hal_bit_t *Dout32;hal_bit_t *Dout33;hal_bit_t *Dout34;
  hal_s32_t retval;
  hal_float_t looptime;hal_bit_t *Din20; hal_bit_t *Din21; hal_bit_t *Din22; hal_bit_t *Din23; hal_bit_t *Din24; hal_bit_t *Din25; hal_bit_t *Din26;
  hal_bit_t *Din27; hal_bit_t *Din28; hal_bit_t *Din29; hal_bit_t *Din210; hal_bit_t *Din211; hal_bit_t *Din212; hal_bit_t *Din213; hal_bit_t *Din214;
  hal_bit_t *Din215; hal_bit_t *Din30;hal_bit_t *Din31;hal_bit_t *Din32;hal_bit_t *Din33;hal_bit_t *Din34;
} haldata_t;
static  int regarg[4];
static int done;
char *modname = "arduino";
static void quit(int sig) {
    done = 1;
}
int write_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *haldata) {  //==============  write  ====================
     int retval;
     int  v0 ,v3;
     v0 = (*(haldata->Dout1)) | (*(haldata->Dout2))<<1| (*(haldata->Dout3))<<2| (*(haldata->Dout4)<<4)| (*(haldata->Dout8))<<7|     
      (*(haldata->Dout9))<<8| (*(haldata->Dout12)<<11); 
       
     retval = preset_single_register(param, 1, 0, v0); //посылаем в устройство1, регистр 0 число v0
     preset_single_register(param, 1, 2, v0);
     //шим 
     preset_single_register(param, 1, 13, abs((int)(*(haldata->pwm1))));   //(13,15,16,19,20,21)
     preset_single_register(param, 1, 15, abs((int)(*(haldata->pwm2))));   
     preset_single_register(param, 1, 16, abs((int)(*(haldata->pwm3))));
     preset_single_register(param, 1, 19, abs((int)(*(haldata->pwm4))));
     preset_single_register(param, 1, 20, abs((int)(*(haldata->pwm5))));
     preset_single_register(param, 1, 21, abs((int)(*(haldata->pwm6))));
     
     if(regarg[0]==0){
     v3 = (*(haldata->Dout30)) | (*(haldata->Dout31))<<1 | (*(haldata->Dout32))<<2 | (*(haldata->Dout33)<<4) | (*(haldata->Dout34))<<7 ;
     preset_single_register(param, 1, 3, v3);
     }
     
    return retval;
}
int read_data(modbus_param_t *param, slavedata_t *slavedata, haldata_t *hal_data_block) {
    int receive_data[MAX_READ_HOLD_REGS];	
    int retval;
    int Reg1val=0;
    int Reg2val=0;
    int mass2[16],mass3[5];
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
    //----------------------------------------------------------------------------------------читаем  цифровые входы --------регистр2
        retval = read_holding_registers(param, slavedata->slave, 2 , 1 , receive_data);
                  
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg1val  = receive_data[0] ; //читаем регистр2
        //printf("Reg1val: %i\n", Reg1val); 

        while ( x < 16  ){
		if ((Reg1val & 0x01) == 1 ) 
			mass2[x] = 1;
		else
			mass2[x] = 0;
		Reg1val=Reg1val>>1;
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
//========================================
if(regarg[0]==1){
        retval = read_holding_registers(param, slavedata->slave, 3 , 1 , receive_data);
                  
        if (retval==1) {
        retval = 0;
        hal_data_block->retval = retval;
        if (retval==0) {
        Reg2val  = receive_data[0] ; //читаем регистр3
        //printf("Reg2val: %i\n", Reg2val);
        x=0;
                while ( x < 5  ){
		if ((Reg2val & 0x01) == 1 ) 
			mass3[x] = 1;
		else
			mass3[x] = 0;
		Reg2val=Reg2val>>1;
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

    getopt(argc,argv,"v:");
	regset = optarg; 	
    rt = atoi(regset);
    for(i = 0;i < 4;i++){
	regarg[i] = rt & 0x01;
	rt = rt >> 1;
	}

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
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain0) , hal_comp_id, "%s.Ain0" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain1) , hal_comp_id, "%s.Ain1" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain2) , hal_comp_id, "%s.Ain2" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain3) , hal_comp_id, "%s.Ain3" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain4) , hal_comp_id, "%s.Ain4" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain5) , hal_comp_id, "%s.Ain5" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain6) , hal_comp_id, "%s.Ain6" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain7) , hal_comp_id, "%s.Ain7" , modname); if (retval!=0) goto out_closeHAL;      
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain8) , hal_comp_id, "%s.Ain8" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain9) , hal_comp_id, "%s.Ain9" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain10), hal_comp_id, "%s.Ain10", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain11), hal_comp_id, "%s.Ain11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain12), hal_comp_id, "%s.Ain12", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain13), hal_comp_id, "%s.Ain13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain14), hal_comp_id, "%s.Ain14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->Ain15), hal_comp_id, "%s.Ain15", modname); if (retval!=0) goto out_closeHAL;
    
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm1), hal_comp_id, "%s.pwm1", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm2), hal_comp_id, "%s.pwm2", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm3), hal_comp_id, "%s.pwm3", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm4), hal_comp_id, "%s.pwm4", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm5), hal_comp_id, "%s.pwm5", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->pwm6), hal_comp_id, "%s.pwm6", modname); if (retval!=0) goto out_closeHAL;
        
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout1),   hal_comp_id, "%s.Dout1", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout2),   hal_comp_id, "%s.Dout2", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout3),   hal_comp_id, "%s.Dout3", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout4),   hal_comp_id, "%s.Dout4", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout8),   hal_comp_id, "%s.Dout8", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout9),   hal_comp_id, "%s.Dout9", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout12),  hal_comp_id, "%s.Dout12", modname); if (retval!=0) goto out_closeHAL;
if(regarg[0]==0){    
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout30),   hal_comp_id, "%s.Dout3-0", modname); if (retval!=0) goto out_closeHAL;    
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout31),   hal_comp_id, "%s.Dout3-1", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout32),   hal_comp_id, "%s.Dout3-2", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout33),   hal_comp_id, "%s.Dout3-3", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Dout34),   hal_comp_id, "%s.Dout3-4", modname); if (retval!=0) goto out_closeHAL;   
} 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din20),    hal_comp_id, "%s.Din2-0" , modname); if (retval!=0) goto out_closeHAL;   
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din21),    hal_comp_id, "%s.Din2-1" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din22),    hal_comp_id, "%s.Din2-2" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din23),    hal_comp_id, "%s.Din2-3" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din24),    hal_comp_id, "%s.Din2-4" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din25),    hal_comp_id, "%s.Din2-5" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din26),    hal_comp_id, "%s.Din2-6" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din27),    hal_comp_id, "%s.Din2-7" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din28),    hal_comp_id, "%s.Din2-8" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din29),    hal_comp_id, "%s.Din2-9" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din210),   hal_comp_id, "%s.Din2-10", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din211),   hal_comp_id, "%s.Din2-11", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din212),   hal_comp_id, "%s.Din2-12", modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din213),   hal_comp_id, "%s.Din2-13", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din214),   hal_comp_id, "%s.Din2-14", modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din215),   hal_comp_id, "%s.Din2-15", modname); if (retval!=0) goto out_closeHAL;
if(regarg[0]==1){
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din30),    hal_comp_id, "%s.Din3-0" , modname); if (retval!=0) goto out_closeHAL;    
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din31),    hal_comp_id, "%s.Din3-1" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din32),    hal_comp_id, "%s.Din3-2" , modname); if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din33),    hal_comp_id, "%s.Din3-3" , modname); if (retval!=0) goto out_closeHAL; 
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->Din34),    hal_comp_id, "%s.Din3-4" , modname); if (retval!=0) goto out_closeHAL;
   }
   
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
if(regarg[0]==0){    
    *(haldata->Dout30) = 0;    
    *(haldata->Dout31) = 0;
    *(haldata->Dout32) = 0;
    *(haldata->Dout33) = 0;
    *(haldata->Dout34) = 0;
}    
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
if(regarg[0]==1){
    *(haldata->Din30) = 0;    
    *(haldata->Din31) = 0; 
    *(haldata->Din32) = 0;
    *(haldata->Din33) = 0;
    *(haldata->Din34) = 0;
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
