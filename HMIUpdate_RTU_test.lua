--捻股机项目,HMI触控屏操作,通过无线修改摇篮仓内部数据,以及低频读取更新设备数据,数据通信使用 MODBUS-RTU 格式
--硬件准备: 上海锐智控制器+WIFI-Ashining无线传输装置一套,或者是红外对射传输装置一套, 森克一体机
--相关数据:
--1 当前张力, 2 当年伺服扭力, 3 当前计米长度, 4 当前故障, 5 当前P , 6 当前I ,7 当前D, 
--8 目标张力, 9 最小值, 10 最大值, 11 Imax, 12 maxdelta
--收到报文后要做丢包校验
--@date: 2023-10-09

--CRC16校验所需函数
--https://blog.csdn.net/u013625451/article/details/103241737
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

local Tension = 0
local Torque = 0
local Length = 0
local Error = 0
local twist_Kp = 0
local twist_Ti = 0
local twist_Td = 0
local twist_ref = 0
local twist_Max = 0
local twist_Min = 0
local twist_IMax = 0
local twist_Maxdelta = 0

local handle = RS232Open("ttyS0", 9600)
if handle > 0 then 
  while true do
    Sleep(10)
    --RS232WriteBytes(handle, {0xaa,0xbb,0xcc,0xdd})
    --Sleep(80)
    --作为slave端,读取 master 端发来的读写请求,其中 slave id 为1
    local _ret, _req = RS232ReadBytes(handle, 8, 10)

    if _ret >= 1 then   
TPWrite(_req)
      local data = {_req[1],_req[2],_req[3],_req[4],_req[5],_req[6]}
      local _crc1,_crc2 = _CRC16(data)
      if ( (_crc1 ~= _req[7]) or (_crc2 ~= _req[8])) then
        --CRC校验错误
        --SetAO("Error",401)
        TPWrite("crc wrong")
      else
        if _req[2] == 3 then
          --数据读取

          local start = _req[4]
          local length = _req[6]
          local _response = {1,3,length*2}
          
          local allData = {Tension,Torque,Length,Error,twist_Kp,twist_Ti,twist_Td,twist_ref,twist_Max,twist_Min,twist_IMax,twist_Maxdelta}
          for i = start,start+length - 1,1 do
            local value = allData[i+1]
            _response[3+i*2+1] = bit_rshift(value,8)
            _response[3+i*2+2] = value - bit_rshift(value,8)*256
          end
          

          local _crc1_,_crc2_ = _CRC16(_response)
          _response[3+length*2+1] = _crc1_
          _response[3+length*2+2] = _crc2_
          RS232WriteBytes(handle, _response)
        elseif _req[2] == 6 then
          TPWrite(_req)
          local val = _req[5]*256 + _req[6]   
          if _req[4] == 5 then
            --SetAO("twist_Kp",val)
            twist_Kp = val
          elseif _req[4] == 6 then
            --SetAO("twist_Ti",val)
            twist_Ti = val
          elseif _req[4] == 7 then
            --SetAO("twist_Td",val)  
            twist_Td = val            
          elseif _req[4] == 8 then
            --SetAO("twist_ref",val)
            twist_ref = val
          elseif _req[4] == 9 then
            --SetAO("twist_Max",val)
            twist_Max = val
          elseif _req[4] == 10 then
            --SetAO("twist_Min",val)
            twist_Min = val
          elseif _req[4] == 11 then
            --SetAO("twist_IMax",val)
            twist_IMax = val
          elseif _req[4] == 12 then
            --SetAO("twist_Maxdelta",val)
            twist_Maxdelta = val
          end
          RS232WriteBytes(handle, _req)
        end

      end
    else
      --串口读取超时
      --SetAO("Error",403)
    end
  end
else
  --串口打开失败
  --SetAO("Error",402)
end
 



local function GLOBALDATA_DEFINE()
end
print("The end!")