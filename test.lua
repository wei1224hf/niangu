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
 
function bit_rshift(value,n)
	value = math.modf(value / (2^n))
	return value
end
 
function CRC16(arr)
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
 
 
--调用
local arr = {0,0,0,0,0,0}
arr[1] = 0x01
arr[2] = 0x03
arr[3] = 0x61
arr[4] = 0x00
arr[5] = 0x00
arr[6] = 0x02
 
print(CRC16(arr))
 
--输出: 219	247  (DB F7)


print(Xor(0xEEFF,0xFFFF))