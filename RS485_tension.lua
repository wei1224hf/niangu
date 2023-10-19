--捻股机项目,张力计逻辑处理
--硬件准备: 上海锐智控制器+驰原张力计(485采集)
--收到报文后要做丢包校验
--@date: 2023-10-07

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

local cyclecount = 0
local cycletime = 10
local sts1,pretime = getcurtime()
local handle = RS232Open("ttyS0", baud.num)
if handle > 0 then 
  while true do
    if isSetZero.num == 1 then
      --归零
      RS232WriteBytes(handle, {0x01 ,0x06 ,0x00 ,0x11 ,0x00 ,0x01 ,0x18 ,0x0F})
      isSetZero.num = 0
      Sleep(10)
    end
    cyclecount = cyclecount+1
    --张力读取报文
    RS232WriteBytes(handle, {0x01 ,0x03 ,0x00 ,0x00 ,0x00 ,0x01 ,0x84 ,0x0A})
    Sleep(10)
    local ret2, data2 = RS232ReadBytes(handle, 7, 10)
    if ret2 >= 1 then    
      local data = {data2[1],data2[2],data2[3],data2[4],data2[5]}
      local _crc1,_crc2 = _CRC16(data)
      if ( (_crc1 ~= data[6]) or (_crc2 ~= data[7])) then
        --CRC校验错误
        ErrorT.num = 1
      else
        local d1 = data2[4] * 256
        local d2 = data2[5]
        local d = d1 + d2
        SetAO("Ao_1",d)
      end
    else
      --串口读取超时
      ErrorT.num = 3
    end
  end
else
  --串口打开失败
  ErrorT.num = 2
end
 
