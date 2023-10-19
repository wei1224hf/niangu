--捻股机项目,HMI触控屏操作,通过无线修改摇篮仓内部数据,以及低频读取更新设备数据,数据通信使用 MODBUS-RTU 格式
--硬件准备: 上海锐智控制器+WIFI-Ashining无线传输装置一套,或者是红外对射传输装置一套, 森克一体机
--收到报文后要做丢包校验
--@date: 2023-10-09

local bit = require("bit")



local Tension = 0
local Length = 0
local SvwireActualPos1 = 0
local SvwireActualSpeed1 = 0
local SvwireActualTorque1 = 0
local SvwirePosition1 = 0
local SvwireTargetSpeed1 = 0
local SvwireTargetTorque1 = 0

local twist_Error = 0
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
local SvwireTargetSpeed2 = 0

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

local prt_cnt = 0



local names = {"twist_Kp","twist_Ti","twist_Td","twist_ref","twist_Max","twist_Min","twist_IMax","twist_Maxdelta","Svwireerror1","SvwireCtrlWord1","SvwireStatusWord1","SvwireOperationMode1","Tension_lo","Tension_hi","Length_lo","Length_hi","SvwireActualSpeed1_lo","SvwireActualSpeed1_hi","SvwireActualTorque1_lo","SvwireActualTorque1_hi","SvwireTargetSpeed1_lo","SvwireTargetSpeed1_hi","SvwireTargetTorque1_lo","SvwireTargetTorque1_hi","SvwireTargetSpeed2_lo","SvwireTargetSpeed2_hi"}


--取数据的低16位跟16高位
function getLowHigh(value)
  if value == nil then return 0,0 end
  local low = 0
  local high = 0
  if value < 0 then
    value = value*(-1) - 1
    local _v = bit.bxor(value,0xffffffff)
    low = bit.band(_v,0xffff)
    high = bit.band(bit.rshift(_v,16),0xffff)
  elseif value> 0xffff then
    low = bit.band(value,0xffff)
    high = bit.rshift( value,16)
  else
    low = value
  end
  return low,high
end

function TPWrite2(value)
  prt_cnt = prt_cnt + 1
  if prt_cnt == 150 then
    TPWrite(value)
    prt_cnt = 0
  end
end


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
      local _crc = CRC16(data)
      local _h = bit.rshift(_crc,8)
      local _l = bit.band(_crc,0xff)

      if ( (_l ~= _req[7]) or (_h ~= _req[8])) then
        --CRC校验错误
        SetAO("twist_Error",401)
      else
        if _req[2] == 3 then
          --数据读取

          TPWrite2(_req)
          
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
          SvwireStatusWord1 = GetAI("SvwireStatusWord1")
          SvwireOperationMode1 = GetAO("SvwireOperationMode1")
          Tension = GetAO("Tension")
          Length = GetAO("Length")          
          SvwireActualSpeed1 = GetAI("SvwireActualSpeed1")
          SvwireActualTorque1 = GetAI("SvwireActualTorque1")
          SvwireTargetSpeed1 = GetAO("SvwireTargetSpeed1")
          SvwireTargetTorque1 = GetAO("SvwireTargetTorque1")  
          --SvwireTargetSpeed2 = GetAO("SvwireTargetSpeed2")          
          
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
          
          local allData = {twist_Kp ,twist_Ti ,twist_Td ,twist_ref ,twist_Max ,twist_Min ,twist_IMax ,twist_Maxdelta ,Svwireerror1 ,SvwireCtrlWord1 ,SvwireStatusWord1 ,SvwireOperationMode1 , Tension_lo,Tension_hi ,Length_lo,Length_hi ,SvwireActualSpeed1_lo,SvwireActualSpeed1_hi ,SvwireActualTorque1_lo,SvwireActualTorque1_hi ,SvwireTargetSpeed1_lo,SvwireTargetSpeed1_hi ,SvwireTargetTorque1_lo,SvwireTargetTorque1_hi,SvwireTargetSpeed2_lo,SvwireTargetSpeed2_hi }
          for i = start,start+length -1,1 do
            local value = allData[i+1]
            _response[3+i*2+1] = bit.rshift(value,8)
            _response[3+i*2+2] = bit.band(value,0xff)
          end
          
          local _crc2 = CRC16(_response)
          local _h2 = bit.rshift(_crc2,8)
          local _l2 = bit.band(_crc2,0xff)

          _response[3+length*2+1] = _l2
          _response[3+length*2+2] = _h2
          TPWrite2(_response)

          RS232WriteBytes(handle, _response)
        elseif _req[2] == 6 then
          local val = 0
          val = _req[5]*256 + _req[6]   
          local _name = names[_req[4]+1]
          if (_name == "Svwireerror1") or (_name == "SvwireActualSpeed1_lo") or (_name == "SvwireActualSpeed1_hi") or (_name == "SvwireActualTorque1_lo") or (_name == "SvwireActualTorque1_hi") then
            TPWrite2(_name)
          elseif (_name == "SvwireTargetTorque1_lo")  then
            SvwireTargetTorque1_lo_ = val
          elseif (_name == "SvwireTargetSpeed2_lo")  then
            SvwireTargetSpeed2_lo_ = val
          elseif (_name == "Length_lo")  then
            Length_lo = val   
          elseif (_name == "SvwireTargetTorque1_hi")  then
            local _val = 0
            if val > 0x8000 then
              local valBig = (val * 0x10000) + SvwireTargetTorque1_lo_
              local valBig_ = bit.bxor(valBig,0xffffffff)
              local _val = valBig_ * (-1) -1
            else
              _val = (val * 0x10000) + SvwireTargetTorque1_lo_
            end
            SetAO("SvwireTargetTorque1",_val)
          elseif (_name == "SvwireTargetSpeed2_hi")  then
            local _val = 0
            if val > 0x8000 then
              local valBig = (val * 0x10000) + SvwireTargetSpeed2_lo_
              local valBig_ = bit.bxor(valBig,0xffffffff)
              local _val = valBig_ * (-1) -1
            else
              _val = (val * 0x10000) + SvwireTargetSpeed2_lo_
            end
            SetAO("SvwireTargetSpeed2",_val)
            
          elseif (_name == "Length_hi")  then
            local _val = 0
            if val > 0x8000 then
              local valBig = (val * 0x10000) + Length_lo
              local valBig_ = bit.bxor(valBig,0xffffffff)
              local _val = valBig_ * (-1) -1
            else
              _val = (val * 0x10000) + Length_lo
            end
            SetAO("Length",_val)     
          else          
            SetAO(_name,val)
          end
          
          RS232WriteBytes(handle, _req)
        end

      end
    else
      --串口读取超时
      --SetAO("twist_Error",403)
    end
  end
else
  --串口打开失败
  SetAO("twist_Error",402)
end