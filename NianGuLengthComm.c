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

int Tension_lo = 0
int Length_lo = 0
int SvwireActualPos1_lo = 0
int SvwireActualSpeed1_lo = 0
int SvwireActualTorque1_lo = 0
int SvwirePosition1_lo = 0
int SvwireTargetSpeed1_lo = 0
int SvwireTargetTorque1_lo = 0
int SvwireTargetSpeed2_lo = 0


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

void modbusComm(){
	
	uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];// request buffer
	int len;// length of the request/response

	while(1) {
		len = modbus_receive(ctx, req);
		if (len == -1) break;

		if(req[1]==3){
			int val = 0;
			int idx = 0
			int lo = 0;
			int hi = 0;
			io_fast_read("twist_Start", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_SetSpeed", &val); mapping->tab_registers[idx] = val; idx ++;	
			io_fast_read("twist_SetLength", &val); mapping->tab_registers[idx] = val; idx ++;	
			io_fast_read("twist_ErrorStop", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_Kp", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_Ti", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_Td", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_ref", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_Max", &val); mapping->tab_registers[idx] = val; idx ++;	

			io_fast_read("twist_Min", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_IMax", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("twist_Maxdelta", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("Svwireerror1", &val); mapping->tab_registers[idx] = val; idx ++;
			io_fast_read("SvwireCtrlWord1", &val); mapping->tab_registers[idx] = val; idx ++;	
			io_fast_read("SvwireOperationMode1", &val); mapping->tab_registers[idx] = val; idx ++;

			io_fast_read("Tension", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;
			io_fast_read("Length", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;
			io_fast_read("SvwireActualSpeed1", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;
			io_fast_read("SvwireActualTorque1", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;
			io_fast_read("SvwireTargetSpeed1", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;
			io_fast_read("SvwireTargetTorque1", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;			
			io_fast_read("SvwireTargetSpeed2", &val); hi = val << 8; lo = val - hi * 65536;  mapping->tab_registers[idx] = lo; idx ++; mapping->tab_registers[idx] = hi; idx ++;			

			
			io_fast_read("restart", &val); mapping->tab_registers[idx] = val;

		}
		else if(req[1]==6){
			int data_address = (req[2] << 8) | req[3];
			int data_value = (req[4] << 8) | req[5];
			
			if(data_address==0){      io_fast_write("twist_Start", data_value); }
			else if(data_address==1){ io_fast_write("twist_SetSpeed", data_value); }
			else if(data_address==2){ io_fast_write("twist_SetLength", data_value); }	
			else if(data_address==3){   }			
			else if(data_address==4){ io_fast_write("twist_Kp", data_value); }			
			else if(data_address==5){ io_fast_write("twist_Ti", data_value); }			
			else if(data_address==6){ io_fast_write("twist_Td", data_value); }			
			else if(data_address==7){ io_fast_write("twist_ref", data_value); }		
			else if(data_address==8){ io_fast_write("twist_Max", data_value); }	
			else if(data_address==9){ io_fast_write("twist_Min", data_value); }	
			else if(data_address==10){ io_fast_write("twist_IMax", data_value); }	
			else if(data_address==11){ io_fast_write("twist_Maxdelta", data_value); }	
			else if(data_address==12){  }	
			else if(data_address==13){ io_fast_write("SvwireCtrlWord1", data_value); }	
			else if(data_address==14){ Tension_lo = data_value; }		
			else if(data_address==15){ io_fast_write("Tension", data_value*65536 + Tension_lo ); }		
			else if(data_address==16){ Length_lo = data_value; }		
			else if(data_address==17){ io_fast_write("Length", data_value*65536 + Length_lo ); }	
			else if(data_address==18){ SvwireActualSpeed1_lo = data_value; }		
			else if(data_address==19){  }	
			else if(data_address==20){ SvwireActualTorque1_lo = data_value; }		
			else if(data_address==21){  }		
			else if(data_address==22){ SvwireTargetSpeed1_lo = data_value; }		
			else if(data_address==23){ io_fast_write("SvwireTargetSpeed1", data_value*65536 + SvwireTargetSpeed1_lo ); }	
			else if(data_address==24){ SvwireTargetTorque1_lo = data_value; }		
			else if(data_address==25){ io_fast_write("SvwireTargetTorque1", data_value*65536 + SvwireTargetTorque1_lo ); }	
			else if(data_address==26){ SvwireTargetSpeed2_lo = data_value; }		
			else if(data_address==27){  }
			else if(data_address==28){ io_fast_write("restart", data_value); }			
		}
		
		len = modbus_reply(ctx, req, len, mapping);
		if (len == -1) break;		
	}
	printf("Exit the loop: %s\n", modbus_strerror(errno));

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

	//Prepare a Modbus mapping with 30 holding registers
	//(plus no output coil, one input coil and two input registers)
	//This will also automatically set the value of each register to 0
	modbus_mapping_t *mapping = modbus_mapping_new(0, 1, 30, 2);
	if (!mapping) {
		fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
		exit(1);
	}


	for(int i=0;i<30;i++){
		mapping->tab_registers[i] = 0;
	}
	

    // Create a Modbus RTU context
    ctx = modbus_new_rtu(port, 115200, 'N', 8, 1);
	if (!ctx) {	
		fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
		exit(1);
	}

    // Set slave ID
    modbus_set_slave(ctx, 1);

    // Connect to the serial port
    if (modbus_connect(ctx) == -1) {
      modbus_free(104);
    }
	

	
    isRunning = true;


}

int main(){
  init();
  run();    


  return 0;  
}
