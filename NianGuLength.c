//#define _XOPEN_SOURCE 700  // This macro enables POSIX.1-2008 features, including usleep
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h> 
#include <modbus/modbus.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sys.h>
#include <record.h>
#include <printk.h>

#define CT					4

bool isRunning = false;
float length = 0;
const char *AO_LENGTH = "Length";
const char *AO_ERROR = "twist_Error";
const char *DI_MI_1 = "mi1";
const char *DI_MI_2 = "mi2";
int mi1 = 0;
int mi2 = 0;
int _mi1 = -99;
int _mi2 = -99;
int Up2 = 0;
int Up1 = 0;
int Down2 = 0;
int Down1 = 0;
float pi = 3.14159265358979323;
float radius = 60.0;


void _Error(int num){
    io_fast_write(AO_ERROR, num);
}

void SetAO(const char *name,int value){
    io_fast_write(name, value);
}

int GetDI(const char *name){
    int val = 0;
    io_fast_read(name, &val);
    return val;
}

int GetAO(const char *name){
    int val = 0;
    io_fast_read(name, &val);
    return val;
}


//依靠IO板上的2个DI信号跟一个计米轮来计米
void readLength(){
    _mi1 = mi1;
    _mi2 = mi2;
    mi1 = GetDI(DI_MI_1);
    mi2 = GetDI(DI_MI_2);
    length = (float)(GetAO(AO_LENGTH));
    bool isChanged = false;

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
      int num = GetAO("mi_C00_10");
      num ++;
      SetAO("mi_C00_10",num);
      isChanged = true;
    }
    
    if ((_mi1==0) && (_mi2==0) && (mi1==0) && (mi2==1)) {
      //顺时针
      length = length + 2*pi*radius/8;
      int num = GetAO("mi_C00_01");
      num ++;
      SetAO("mi_C00_01",num);
      isChanged = true;
    }  
    
    //两个都是0,瞬变到两个都是1,则中间状态丢失
    if ((_mi1==0) && (_mi2==0) && (mi1==1) && (mi2==1)) {
        _Error(201);
      int num = GetAO("mi_C00_11");
      num ++;
      SetAO("mi_C00_11",num);
    }   
    
    if ((_mi1==1) && (_mi2==0) && (mi1==1) && (mi2==1)) {
      //逆时针
      length = length - 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C10_11");
      num ++;
      SetAO("mi_C10_11",num);
    } 

    if ((_mi1==1) && (_mi2==0) && (mi1==0) && (mi2==0)) {
      //顺时针
      length = length + 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C10_00");
      num ++;
      SetAO("mi_C10_00",num); 
    }
    
    //1亮2灭,瞬变到1灭2亮,则中间状态丢失
    if ((_mi1==1) && (_mi2==0) && (mi1==0) && (mi2==1)) {
      _Error(202);
      int num = GetAO("mi_C10_01");
      num ++;
      SetAO("mi_C10_01",num); 
    }  
    
    if ((_mi1==0) && (_mi2==1) && (mi1==0) && (mi2==0)) {
      length = length - 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C01_00");
      num ++;
      SetAO("mi_C01_00",num); 
    }  
    
    if ((_mi1==0) && (_mi2==1) && (mi1==1) && (mi2==1)) {
      length = length + 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C01_11");
      num ++;
      SetAO("mi_C01_11",num); 
    }  
    
    //1灭2亮,瞬变到1亮2灭,则中间状态丢失
    if ((_mi1==0) && (_mi2==1) && (mi1==1) && (mi2==0)) {
      _Error(203);
      int num = GetAO("mi_C01_10");
      num ++;
      SetAO("mi_C01_10",num); 
    } 
    
    if ((_mi1==1) && (_mi2==1) && (mi1==0) && (mi2==1)) {
      length = length - 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C11_01");
      num ++;
      SetAO("mi_C11_01",num); 
    } 
    
    if ((_mi1==1) && (_mi2==1) && (mi1==1) && (mi2==0)) {
      length = length + 2*pi*radius/8;
      isChanged = true;
      int num = GetAO("mi_C11_10");
      num ++;
      SetAO("mi_C11_10",num); 
    } 
    
    //1亮2亮,瞬变到1灭2灭,则中间状态丢失
    if ((_mi1==1) && (_mi2==1) && (mi1==0) && (mi2==0)) {
      _Error(204);
      int num = GetAO("mi_C11_00");
      num ++;
      SetAO("mi_C11_00",num); 
    } 

    if(isChanged){
      int _length = (int)length;
      SetAO(AO_LENGTH,_length);
    }
}

int run(){
    DWORD t1 = 0;
    DWORD t2 = 0;
    while(isRunning){
        t1 = GetTickCount();
        readLength();
        t2 = GetTickCount();
        os_delay(CT - (t2-t1));
    }
    return 1;

}


void init() {
    length = 0;
	
    const char *port = "/dev/ttyS1"; 


    // Create a Modbus RTU context
    ctx = modbus_new_rtu(port, 115200, 'N', 8, 1);
    if (ctx == NULL) {
      _Error(103);
    }

    // Set slave ID
    modbus_set_slave(ctx, 1);

    // Connect to the serial port
    if (modbus_connect(ctx) == -1) {
      _Error(104);
    }
    isRunning = true;


}

int main(){
  init();
  run();    


  return 0;  
}
