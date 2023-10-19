//#define _XOPEN_SOURCE 700  // This macro enables POSIX.1-2008 features, including usleep
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h> 
#include <modbus/modbus.h>
#include <time.h>

bool isRunning = false;
modbus_t *ctx = NULL;
int slave_id = 1; // Modbus RTU slave ID
int tension = 0;
float length = 0;
uint16_t ADDRESS_TENSION = 0x00;
uint16_t ADDRESS_SETZERO = 0x11;
bool IS_SET_ZERO = false;
const char *AO_TENSION = "Ao_1";
const char *AO_LENGTH = "Ao_2";
const char *AO_ERROR = "Ao_3";
const char *DI_MI_1 = "DI_1";
const char *DI_MI_2 = "DI_2";
int mi1 = 0;
int mi2 = 0;
int _mi1 = -99;
int _mi2 = -99;
int Up2 = 0;
int Up1 = 0;
int Down2 = 0;
int Down1 = 0;
float pi = 3.1415926;
float radius = 60.0;
int _ERROR = 0;
int delay = 20;
int pOutCount = 0;
int pOutMax = 30;


void _error(const char *msg, modbus_t *ctx) {
     if (ctx != NULL) {
        modbus_free(ctx);
     }
     perror(msg);
     SetAO(AO_ERROR,_ERROR);
     exit(EXIT_FAILURE);
}

void SetAO(const char *name,int value){
    //TODO
}

int GetDI(const char *name){
    //TODO
    return 1;
}

//从张力计上读取张力,基于 MODBUS-RTU 通信
void readTension(){
    if(IS_SET_ZERO){
        //对张力计作零点设置
        uint16_t value_to_write = 1;
        if (modbus_write_register(ctx, ADDRESS_SETZERO, value_to_write) == -1) {
            _ERROR = 101;
            _error("Error writing to Modbus register",ctx);
            
        }
    }
    
    //读取张力
    uint16_t read_value;
    if (modbus_read_registers(ctx, ADDRESS_TENSION, 1, &read_value) == -1) {
        _ERROR = 102;
        _error("Error reading from Modbus register", ctx);
        
    }    
    if(pOutCount >= pOutMax){ 
        printf("Value read from Modbus register: %u\n", read_value);
        pOutCount = 0;
    }
    else{
        pOutCount = pOutCount + 1;
    }
    if(read_value>=0x8000){
        read_value = 0;
    }
    SetAO(AO_TENSION,read_value);
}

//依靠IO板上的2个DI信号跟一个计米轮来计米
void readLength(){
    _mi1 = mi1;
    _mi2 = mi2;
    mi1 = GetDI(DI_MI_1);
    mi2 = GetDI(DI_MI_2);

    //累计2号计米信号的亮灭次数,用于debug信号丢失
    if ((_mi2==false) && (mi2==true)) {
      Up2 = Up2 + 1;
    }
    else if ((_mi2==true) && (mi2==false)) {
      Down2 = Down2 + 1;
    }    

    //累计1号计米信号的亮灭次数,用于debug信号丢失
    if ((_mi1==0) && (mi1==1)) {
      Up1 = Up1 + 1;
    }
    else if((_mi1==1) && (mi1==0)) {
      Down1 = Down1 + 1;
    }
    
    //转动一圈,需要8次信号变化
    if ((_mi1==0) && (_mi2==0) && (mi1==1) && (mi2==0)) {
      //逆时针
      length = length - 2*pi*radius/8;
    }
    
    if ((_mi1==0) && (_mi2==0) && (mi1==0) && (mi2==1)) {
      //顺时针
      length = length + 2*pi*radius/8;
    }  
    
    //两个都是0,瞬变到两个都是1,则中间状态丢失
    if ((_mi1==0) && (_mi2==0) && (mi1==1) && (mi2==1)) {
     _ERROR = 201;
    }   
    
    if ((_mi1==1) && (_mi2==0) && (mi1==1) && (mi2==1)) {
      //逆时针
      length = length - 2*pi*radius/8;
    } 

    if ((_mi1==1) && (_mi2==0) && (mi1==0) && (mi2==0)) {
      //顺时针
      length = length + 2*pi*radius/8;
    }
    
    //1亮2灭,瞬变到1灭2亮,则中间状态丢失
    if ((_mi1==1) && (_mi2==0) && (mi1==0) && (mi2==1)) {
      _ERROR = 201;
    }  
    
    if ((_mi1==0) && (_mi2==1) && (mi1==0) && (mi2==0)) {
      length = length - 2*pi*radius/8;
    }  
    
    if ((_mi1==0) && (_mi2==1) && (mi1==1) && (mi2==1)) {
      length = length + 2*pi*radius/8;
    }  
    
    //1灭2亮,瞬变到1亮2灭,则中间状态丢失
    if ((_mi1==0) && (_mi2==1) && (mi1==1) && (mi2==0)) {
      _ERROR = 203;
    } 
    
    if ((_mi1==1) && (_mi2==1) && (mi1==0) && (mi2==1)) {
      length = length - 2*pi*radius/8;
    } 
    
    if ((_mi1==1) && (_mi2==1) && (mi1==1) && (mi2==0)) {
      length = length + 2*pi*radius/8;
    } 
    
    //1亮2亮,瞬变到1灭2灭,则中间状态丢失
    if ((_mi1==1) && (_mi2==1) && (mi1==0) && (mi2==0)) {
      _ERROR = 204;
    } 

    int _length = (int)length;
    SetAO(AO_LENGTH,_length);
}

int run(){
    
    while(isRunning){
        readTension();
        readLength();
        usleep(delay*1000);
    }
    return 1;

}


void init() {
    const char *port = "/dev/ttyS0"; 


    // Create a Modbus RTU context
    ctx = modbus_new_rtu(port, 115200, 'N', 8, 1);
    if (ctx == NULL) {
        _ERROR = 103;
        _error("Unable to create Modbus context", NULL);
    }

    // Set slave ID
    modbus_set_slave(ctx, slave_id);

    // Connect to the serial port
    if (modbus_connect(ctx) == -1) {
        _ERROR = 104;      
        _error("Modbus connection failed", ctx);
    }
    isRunning = true;


}

int main(){
  init();
  run();    

  // Disconnect && free the context
  modbus_close(ctx);
  modbus_free(ctx);

  return 0;  
}