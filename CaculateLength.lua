--捻股机项目,计米器逻辑处理
--硬件准备: 上海锐智控制器+IO板+2个金属感应开关(DI)+一个铁质计米圆盘
--难点处理: 在排线换向的时候,线束的张力会发生较大变化,可能导致计米盘来回震荡一次.
--相关参数: 线速度最高80米/分,计米轮半径60mm,计米轮最高转速 4圈/秒, 转动一圈触发8个DI信号变化, 程序周期必须在 30ms 以内. 如果程序发现过程逻辑信号错误,就报错退出
--@author: 魏兴峰,18989304974,weixf@emergen.cn
--@date: 2023-10-07

local mi1 = GetDI("mi1")
local mi2 = GetDI("mi2")
local _mi1 = -99
local _mi2 = -99
local pi = 3.1415926

--故障信号清零,允许外部清零
ErrorCL.num = 0
--计米清零,允许外部清零
length.num = 0

while true do
  _mi1 = mi1
  _mi2 = mi2
  mi1 = GetDI("mi1")
  mi2 = GetDI("mi2")
  
  --无故障,才能执行计米程序
  if ErrorCL.num == 0 then

    --累计2号计米信号的亮灭次数,用于debug信号丢失
    if ((_mi2==0) and (mi2==1)) then
      Up2.num = Up2.num + 1
    elseif ((_mi2==1) and (mi2==0)) then
      Down2.num = Down2.num + 1
    end
    
    --累计1号计米信号的亮灭次数,用于debug信号丢失
    if ((_mi1==0) and (mi1==1)) then
      Up1.num = Up1.num + 1
    elseif ((_mi1==1) and (mi1==0)) then
      Down1.num = Down1.num + 1
    end
    
    --转动一圈,需要8次信号变化
    if ((_mi1==0) and (_mi2==0) and (mi1==1) and (mi2==0)) then
      --逆时针
      length.num = length.num - 2*pi*radius.num/8
    end
    
    if ((_mi1==0) and (_mi2==0) and (mi1==0) and (mi2==1)) then
      --顺时针
      length.num = length.num + 2*pi*radius.num/8
    end  
    
    --两个都是0,瞬变到两个都是1,则中间状态丢失
    if ((_mi1==0) and (_mi2==0) and (mi1==1) and (mi2==1)) then
      ErrorCL.num = 1
    end   
    
    if ((_mi1==1) and (_mi2==0) and (mi1==1) and (mi2==1)) then
      --逆时针
      length.num = length.num - 2*pi*radius.num/8
    end 

    if ((_mi1==1) and (_mi2==0) and (mi1==0) and (mi2==0)) then
      --顺时针
      length.num = length.num + 2*pi*radius.num/8
    end
    
    --1亮2灭,瞬变到1灭2亮,则中间状态丢失
    if ((_mi1==1) and (_mi2==0) and (mi1==0) and (mi2==1)) then
      ErrorCL.num = 2
    end  
    
    if ((_mi1==0) and (_mi2==1) and (mi1==0) and (mi2==0)) then
      length.num = length.num - 2*pi*radius.num/8
    end  
    
    if ((_mi1==0) and (_mi2==1) and (mi1==1) and (mi2==1)) then
      length.num = length.num + 2*pi*radius.num/8
    end  
    
    --1灭2亮,瞬变到1亮2灭,则中间状态丢失
    if ((_mi1==0) and (_mi2==1) and (mi1==1) and (mi2==0)) then
      ErrorCL.num = 3
    end 
    
    if ((_mi1==1) and (_mi2==1) and (mi1==0) and (mi2==1)) then
      length.num = length.num - 2*pi*radius.num/8
    end 
    
    if ((_mi1==1) and (_mi2==1) and (mi1==1) and (mi2==0)) then
      length.num = length.num + 2*pi*radius.num/8
    end 
    
    --1亮2亮,瞬变到1灭2灭,则中间状态丢失
    if ((_mi1==1) and (_mi2==1) and (mi1==0) and (mi2==0)) then
      ErrorCL.num = 4
    end 

  end
  
  Sleep(length_cycle.num)
end



