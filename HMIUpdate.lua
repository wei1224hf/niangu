--捻股机项目,HMI触控屏操作,通过无线修改摇篮仓内部数据,以及低频读取更新设备数据,数据通信使用 MODBUS-RTU 格式
--硬件准备: 上海锐智控制器+WIFI-Ashining无线传输装置一套,或者是红外对射传输装置一套, 森克一体机
--相关数据:
--1 当前张力, 2 当年伺服扭力, 3 当前计米长度, 4 当前故障, 5 当前P , 6 当前I ,7 当前D, 
--8 目标张力, 9 最小值, 10 最大值, 11 Imax, 12 maxdelta
--收到报文后要做丢包校验
--@date: 2023-10-09

--CRC16校验所需函数
--https://blog.csdn.net/u013625451/article/details/103241737
local bit = require("bit")

function And(num1,num2)
	local tmp1 = num1
	local tmp2 = num2
	local ret = 0
	local count = 0
	repeat
		local s1 = tmp1 % 2
		local s2 = tmp2 % 2
		if s1 == s2 and s1 == 1 then
			ret = ret + 2^count
		end
		tmp1 = math.modf(tmp1/2)
		tmp2 = math.modf(tmp2/2)
		count = count + 1
	until(tmp1 == 0 and tmp2 == 0)
	return ret
end
--CRC16校验所需函数
function Xor(num1,num2)
	local tmp1 = num1
	local tmp2 = num2
	local ret = 0
	local count = 0
	repeat
		local s1 = tmp1 % 2
		local s2 = tmp2 % 2
		if s1 ~= s2 then
			ret = ret + 2^count
		end
		tmp1 = math.modf(tmp1/2)
		tmp2 = math.modf(tmp2/2)
		count = count + 1
	until(tmp1 == 0 and tmp2 == 0)
	return ret
end
--CRC16校验所需函数
function bit_rshift(value,n)
	value = math.modf(value / (2^n))
	return value
end
--CRC16校验所需函数
function _CRC16(arr)
	local tmp = 0xffff
	for i=1,#arr do
		tmp = Xor(arr[i],tmp)
		for j=1,8 do
			local tmp1 = And(tmp,0x01)
			if tmp1 == 1 then
				tmp = bit_rshift(tmp,1)
				tmp = Xor(tmp,0xa001)
			else
				tmp = bit_rshift(tmp,1)
			end
		end
	end
	local ret1 = (tmp % 256)
	local ret2 = math.modf( tmp / 256)
	return ret1,ret2
end
--取数据的低8位跟8高位
function getLowHigh(value)
  local low = 0
  local high = 0
  if value < 0 then
    local lo = string.sub(string.gsub(string.format("0x%16X",value)," ","0"),14,18)
    local hi = string.sub(string.gsub(string.format("0x%16X",value)," ","0"),10,13)
    low = tonumber("0x"+lo)
    high = tonumber("0x"+hi)
  elseif value> 0xffff then
    low = bit.rshift( bit.lshift( value,16),16)
    high = bit.lshift( value,16)
  end
  return low,high
end

local Tension = 0
local Length = 0
local SvwireActualPos1 = 0
local SvwireActualSpeed1 = 0
local SvwireActualTorque1 = 0
local SvwirePosition1 = 0
local SvwireTargetSpeed1 = 0
local SvwireTargetTorque1 = 0

local Error = 0
local twist_Kp = 0
local twist_Ti = 0
local twist_Td = 0
local twist_ref = 0
local twist_Max = 0
local twist_Min = 0
local twist_IMax = 0
local twist_Maxdelta = 0
local Svwireerror1 = 0
local SvwireOperationMode1 = 0
local SvwireCtrlWord1 = 0
local SvwireStatusWord1 = 0

local Tension_lo = 0
local Length_lo = 0
local SvwireActualPos1_lo = 0
local SvwireActualSpeed1_lo = 0
local SvwireActualTorque1_lo = 0
local SvwirePosition1_lo = 0
local SvwireTargetSpeed1_lo = 0
local SvwireTargetTorque1_lo = 0
local SvwireTargetSpeed2_lo = 0

local Tension_hi = 0
local Length_hi = 0
local SvwireActualPos1_hi = 0
local SvwireActualSpeed1_hi = 0
local SvwireActualTorque1_hi = 0
local SvwirePosition1_hi = 0
local SvwireTargetSpeed1_hi = 0
local SvwireTargetTorque1_hi = 0
local SvwireTargetSpeed2_hi = 0

local SvwireTargetSpeed2_lo_ = 0
local SvwireTargetTorque1_lo_ = 0



local names = {"twist_Ti","twist_Td","twist_ref","twist_Max","twist_Min","twist_IMax","twist_Maxdelta","Svwireerror1","SvwireCtrlWord1","SvwireStatusWord1","SvwireOperationMode1","SvwireActualSpeed1","SvwireActualTorque1","SvwireTargetSpeed1","SvwireTargetTorque1","Tension_lo","Tension_hi","Length_lo","Length_hi","SvwireActualSpeed1_lo","SvwireActualSpeed1_hi","SvwireActualTorque1_lo","SvwireActualTorque1_hi","SvwireTargetSpeed1_lo","SvwireTargetSpeed1_hi","SvwireTargetTorque1_lo","SvwireTargetTorque1_hi","SvwireTargetSpeed2_lo","SvwireTargetSpeed2_hi"}



local handle = RS232Open("ttyS1", 115200)
if handle > 0 then 
  while true do
    Sleep(80)
    --RS232WriteBytes(handle, {0xaa,0xbb,0xcc,0xdd})
    --Sleep(80)
    --作为slave端,读取 master 端发来的读写请求,其中 slave id 为1
    local _ret, _req = RS232ReadBytes(handle, 8, 10)
    if _ret >= 1 then    
      local data = {_req[1],_req[2],_req[3],_req[4],_req[5],_req[6]}
      local _crc1,_crc2 = _CRC16(data)
      if ( (_crc1 ~= data[7]) or (_crc2 ~= data[8])) then
        --CRC校验错误
        SetAO("Error",401)
      else
        if _req[2] == 3 then
          --数据读取

          
          twist_Kp = GetAO("twist_Kp")
          twist_Ti = GetAO("twist_Ti")
          twist_Td = GetAO("twist_Td")
          twist_ref = GetAO("twist_ref")
          twist_Max = GetAO("twist_Max")
          twist_Min = GetAO("twist_Min")
          twist_IMax = GetAO("twist_IMax")
          twist_Maxdelta = GetAO("twist_Maxdelta")
          Svwireerror1 = GetAI("Svwireerror1")
          SvwireCtrlWord1 = GetAO("SvwireCtrlWord1")          
          SvwireStatusWord1 = GetAO("SvwireStatusWord1")
          SvwireOperationMode1 = GetAO("SvwireOperationMode1")
          
          Tension = GetAO("Tension")
          Length = GetAO("Length")          
          SvwireActualSpeed1 = GetAI("SvwireActualSpeed1")
          SvwireActualTorque1 = GetAI("SvwireActualTorque1")
          SvwireTargetSpeed1 = GetAO("SvwireTargetSpeed1")
          SvwireTargetTorque1 = GetAO("SvwireTargetTorque1")  
          SvwireTargetSpeed2 = GetAO("SvwireTargetSpeed2")          
          
          Tension_lo,Tension_hi = getLowHigh(Tension)
          Length_lo,Length_hi = getLowHigh(Length)
          SvwireActualSpeed1_lo,SvwireActualSpeed1_hi = getLowHigh(SvwireActualSpeed1)
          SvwireActualTorque1_lo,SvwireActualTorque1_hi = getLowHigh(SvwireActualTorque1)
          SvwireTargetSpeed1_lo,SvwireTargetSpeed1_hi = getLowHigh(SvwireTargetSpeed1)
          SvwireTargetTorque1_lo,SvwireTargetTorque1_hi = getLowHigh(SvwireTargetTorque1)
          SvwireTargetSpeed2_lo,SvwireTargetSpeed2_hi = getLowHigh(SvwireTargetSpeed2)
          


          local start = _req[4]
          local length = _req[6]
          local _response = {1,3,length*2}
          
          
          local allData = {twist_Kp ,twist_Ti ,twist_Td ,twist_ref ,twist_Max ,twist_Min ,twist_IMax ,twist_Maxdelta ,Svwireerror1 ,SvwireCtrlWord1 ,SvwireStatusWord1 ,SvwireOperationMode1 ,SvwireActualSpeed1 ,SvwireActualTorque1 ,SvwireTargetSpeed1 ,SvwireTargetTorque1 ,Tension_lo,Tension_hi ,Length_lo,Length_hi ,SvwireActualSpeed1_lo,SvwireActualSpeed1_hi ,SvwireActualTorque1_lo,SvwireActualTorque1_hi ,SvwireTargetSpeed1_lo,SvwireTargetSpeed1_hi ,SvwireTargetTorque1_lo,SvwireTargetTorque1_hi,SvwireTargetSpeed2_lo,SvwireTargetSpeed2_hi }
          for i = start,start+length -1,1 do
            local value = allData[i+1]
            _response[3+i*2+1] = bit_rshift(value)
            _response[3+i*2+2] = value - bit_rshift(value)*256
          end
          
          local _crc1_,_crc2_ = _CRC16(_response)
          _response[3+length*2+1] = _crc1_
          _response[3+length*2+2] = _crc2_
          RS232WriteBytes(handle, _response)
        elseif _req[2] == 6 then
          local val = 0
          val = _req[5]*256 + _req[6]   
          local _name = names[_req[4]+1]
          if (_name == "Svwireerror1") or (_name == "SvwireActualSpeed1_lo") or (_name == "SvwireActualSpeed1_hi") or (_name == "SvwireActualTorque1_lo") or (_name == "SvwireActualTorque1_hi") then
            TPWrite(_name)
          elseif (_name == "SvwireTargetTorque1_lo")  then
            SvwireTargetTorque1_lo_ = val
          elseif (_name == "SvwireTargetSpeed2_lo")  then
            SvwireTargetSpeed2_lo_ = val
          elseif (_name == "SvwireTargetTorque1_hi")  then
            local _lo = string.format("%4X",SvwireTargetTorque1_lo_)
            _lo = string.gsub(_lo," ","0")
            local _hi = string.format("%4X",val)
            _hi = string.gsub(_hi," ","0")
            local _val = tonumber("0x"+_hi+_lo)
            _val = bit.bxor(0,tonumber(val))
            SetAO("SvwireTargetTorque1",val)
          elseif (_name == "SvwireTargetSpeed2_hi")  then
            local _lo = string.format("%4X",SvwireTargetSpeed2_lo_)
            _lo = string.gsub(_lo," ","0")
            local _hi = string.format("%4X",val)
            _hi = string.gsub(_hi," ","0")
            local _val = tonumber("0x"+_hi+_lo)
            _val = bit.bxor(0,tonumber(val))
            SetAO("SvwireTargetSpeed2",val)
          else          
            SetAO(_name,val)
          end
          
          RS232WriteBytes(handle, _req)
        end

      end
    else
      --串口读取超时
      SetAO("Error",403)
    end
  end
else
  --串口打开失败
  SetAO("Error",402)
end
 
