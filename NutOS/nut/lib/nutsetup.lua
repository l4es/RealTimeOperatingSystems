--
-- Copyright (C) 2012 by egnite GmbH
--
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
-- 3. Neither the name of the copyright holders nor the names of
--    contributors may be used to endorse or promote products derived
--    from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- For additional information see http://www.ethernut.de/
--

archs = {
	arm7tdmi = { "ARM7", "arm7tdmi" },
	arm9 = { "ARM9", "arm9" },
	armcm3 = { "ARM Cortex-M3", "cortex-m3" },
	armcm4 = { "ARM Cortex-M4", "cortex-m4" },
	avr = { "AVR 8-bit" },
	avr32 = { "AVR 32-bit" }
}

devices = {
  at90can128 = { name = "AT90CAN128", arch = archs.avr },
  at90usb1287 = { name = "AT90USB1287", arch = archs.avr },
  at91r40008 = { name = "AT91R40008", arch = archs.arm7tdmi },
  at91sam7s256 = { name = "AT91SAM7S256", arch = archs.arm7tdmi },
  at91sam7se512 = { name = "AT91SAM7SE512", arch = archs.arm7tdmi },
  at91sam7x256 = { name = "AT91SAM7X256", arch = archs.arm7tdmi },
  at91sam7x512 = { name = "AT91SAM7X512", arch = archs.arm7tdmi },
  at91sam9260 = { name = "AT91SAM9260", arch = archs.arm9 },
  at91sam9g45 = { name = "AT91SAM9G45", arch = archs.arm9 },
  at91sam9xe512 = { name = "AT91SAM9XE512", arch = archs.arm9 },
  atmega103 = { name = "ATMEGA103", arch = archs.avr },
  atmega128 = { name = "ATMEGA128", arch = archs.avr },
  atmega2561 = { name = "ATMEGA2561", arch = archs.avr },
  avr32uc3a0512 = { name = "AVR32UC3A0512", arch = archs.avr32 },
  avr32uc3a3256 = { name = "AVR32UC3A3256", arch = archs.avr32 },
  avr32uc3b0256 = { name = "AVR32UC3B0256", arch = archs.avr32 },
  gba = { name = "GBA", arch = archs.arm7tdmi },
  lm3s9b96 = { name = "LM3S9B96", arch = archs.arm_cm3 },
  lpc1768 = { name = "LPC1768", arch = archs.arm_cm3 },
  lpc1778 = { name = "LPC1778", arch = archs.arm_cm3 },
  lpc1788 = { name = "LPC1788", arch = archs.arm_cm3 },
  stm32f100 = { name = "STM32F100", arch = archs.arm_cm3 },
  stm32f103 = { name = "STM32F103", arch = archs.arm_cm3 },
  stm32f107 = { name = "STM32F107", arch = archs.arm_cm3 },
  stm32f205 = { name = "STM32F205", arch = archs.arm_cm3 },
  stm32f207 = { name = "STM32F207", arch = archs.arm_cm3 },
  stm32f405 = { name = "STM32F405", arch = archs.arm_cm3 },
  stm32l151 = { name = "STM32L151", arch = archs.arm_cm4 },
  zero = { name = "ZERO", arch = archs.arm7tdmi },
}

boards = {
  {
    name = { "Arthernet 1", "ARTHERNET1" },
	device = devices.atmega128,
  },
  {
    name = { "Atmel AT90USBKEY", "AT90USBKEY" },
	device = devices.at90usb1287,
  },
  {
    name = { "Atmel AT91EB40A", "AT91EB40A" },
	device = devices.at91r40008,
  },
  {
    name = { "Generic AT91SAM7S256", "AT91SAM7S" },
	device = devices.at91sam7s256,
  },
  {
    name = { "Atmel AT91SAM7SE-EK", "AT91SAM7SE_EK" },
	device = devices.at91sam7se512,
  },
  {
    name = { "Atmel AT91SAM7X-EK", "AT91SAM7X_EK" },
	device = devices.at91sam7x256,
  },
  {
    name = { "Atmel AT91SAM9260-EK", "AT91SAM9260_EK" },
	device = devices.at91sam9260,
  },
  {
    name = { "Atmel AT91SAM9G45-EK", "AT91SAM9G45_EK" },
	device = devices.at91sam9g45,
  },
  {
    name = { "hwgroup Charon II", "CHARON2" },
	device = devices.atmega128,
  },
  {
    name = { "Luminary DK-LM3S9B96", "DK_LM3S9B96" },
	device = devices.lm3s9b96,
  },
  {
    name = { "Elektor Internet Radio", "ELEKTOR_IR1" },
	device = devices.at91sam7se512,
  },
  {
    name = { "Embedded-IT eNet-sam7X", "ENET_SAM7X" },
	device = devices.at91sam7x512,
  },
  {
    name = { "egnite Ethernut 1", "ETHERNUT1" },
	device = devices.atmega128,
  },
  {
    name = { "egnite Ethernut 2", "ETHERNUT2" },
	device = devices.atmega128,
  },
  {
    name = { "egnite Ethernut 3", "ETHERNUT3" },
	device = devices.at91r40008,
  },
  {
    name = { "egnite Ethernut 5", "ETHERNUT5" },
	device = devices.at91sam9xe512,
  },
  {
    name = { "EVK1100", "EVK1100" },
	device = devices.avr32uc3a0512,
  },
  {
    name = { "EVK1101", "EVK1101" },
	device = devices.avr32uc3b0256,
  },
  {
    name = { "EVK1104", "EVK1104" },
	device = devices.avr32uc3a3256,
  },
  {
    name = { "EVK1105", "EVK1105" },
	device = devices.avr32uc3a0512,
  },
  {
    name = { "ST F4 Discovery", "F4_DISCOVERY" },
	device = devices.stm32f405,
  },
  {
    name = { "egnite FLECX", "FLECX1" },
	device = devices.lpc1768,
  },
  {
    name = { "Nintendo GBA", "GBAXPORT2" },
	device = devices.gba,
  },
  {
    name = { "IAR KSK-LPC1788-SK", "KSK_LPC1788_SK" },
	device = devices.lpc1788,
  },
  {
    name = { "Embedded-IT Lisa", "LISA" },
	device = devices.lpc1778,
  },
  {
    name = { "mbed NXP LPC1768", "MBED_NXP_LPC1768" },
	device = devices.lpc1768,
  },
  {
    name = { "Propox MMNET01", "MMNET01" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET02" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET03" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET04" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET101" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET102" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET103" },
	device = devices.atmega128,
  },
  {
    name = { "Propox MMNET01", "MMNET104" },
	device = devices.atmega128,
  },
  {
    name = { "egnite Morphoq", "MORPHOQ1" },
	device = devices.at91sam7x512,
  },
  {
    name = { "egnite SAM7ETH", "SAM7ETH" },
	device = devices.at91sam7x512,
	compilers = {
	  { "GNU C ARM-ELF", "arm-elf" },
	  { "GNU C ARM-EABI", "arm-none-eabi" }
	},
	linker_scripts = {
	  { "Program in Flash", "at91sam7x512_rom" }
	}
  },
  {
    name = { "STEVAL_PCC010V2", "STEVAL_PCC010V2" },
	device = devices.stm32f205,
  },
  {
    name = { "Atmel STK501", "STK501" },
	device = devices.atmega128,
  },
  {
    name = { "STM32_CAN", "STM32_CAN" },
	device = devices.stm32f107,
  },
  {
    name = { "STM32_COMSTICK", "STM32_COMSTICK" },
	device = devices.stm32f107,
  },
  {
    name = { "STM3210C_EVAL", "STM3210C_EVAL" },
	device = devices.stm32f107,
  },
  {
    name = { "STM3210E_EVAL", "STM3210E_EVAL" },
	device = devices.stm32f103,
  },
  {
    name = { "USPS_F107C", "USPS" },
	device = devices.stm32f107,
  },
  {
    name = { "USPS_F205C", "USPS" },
	device = devices.stm32f205,
  },
  {
    name = { "USPS_F405G", "USPS" },
	device = devices.stm32f405,
  },
  {
    name = { "USPS_L151B", "USPS" },
	device = devices.stm32l151,
  },
  {
    name = { "Proconx XNUT 100", "XNUT_100" },
	device = devices.atmega128,
  },
  {
    name = { "Proconx XNUT 105", "XNUT_105" },
	device = devices.at90can128,
  },
  {
    name = { "Zero", "ZERO_EK" },
	device = devices.zero,
  },
}

function get_arch_list()
  local t
  t = {}
  for i, v in ipairs(boards) do
    if v.device ~= nil then
      local found

      found = false
      for ii, vv in pairs(t) do
	    if vv == v.device.arch then
	      found = true
		  break
		end
	  end
	  if not found then
	    t[#t + 1] = v.device.arch
	  end
	end
  end
  return t
end

function get_device_list(arch)
  local t
  t = {}
  for i, v in ipairs(boards) do
    if v.device ~= nil and v.device.arch == arch then
      local found

      found = false
      for ii, vv in pairs(t) do
	    if vv == v.device then
	      found = true
		  break
		end
	  end
	  if not found then
	    t[#t + 1] = v.device
	  end
	end
  end
  return t
end

function get_board_list(device)
  local t
  t = {}
  for i, v in ipairs(boards) do
    if v.device ~= nil and v.device == device then
      t[#t + 1] = v
	end
  end
  return t
end

function get_device_name(device)
  local t
  t = {}
  for i, v in pairs(devices) do
    if v == device then
      return i
	end
  end
  return nil
end

function get_arch_name(arch)
  local t
  t = {}
  for i, v in pairs(archs) do
    if v == arch then
	  if v.name == nil then
		return i
	  else
	    return v.name
	  end
	end
  end
  return nil
end

function create_ifnotexists(path, content)
  local f
  local e
  f,e = io.open(path, "r")
  if e then  
    f = io.open(path, "w")
    f:write(content)  
  end
  f:close()
  return e
end

sel_list = get_arch_list()
io.write("Select a target architecture:\n")
repeat
  for i, v in ipairs(sel_list) do
    io.write(string.format("  %d) %s\n", i, v[1]))
  end
  sel = tonumber(io.read())
  target_arch = sel_list[sel]
  io.write("\n")
until target_arch ~= nil

sel_list = get_device_list(target_arch)
io.write("Select a target device:\n")
repeat
  for i, v in ipairs(sel_list) do
    io.write(string.format("  %d) %s\n", i, v.name))
  end
  sel = tonumber(io.read())
  target_device = sel_list[sel]
  io.write("\n")
until target_device ~= nil

sel_list = get_board_list(target_device)
io.write("Select a target board:\n")
repeat
  for i, v in ipairs(sel_list) do
    io.write(string.format("  %d) %s\n", i, v.name[1]))
  end
  sel = tonumber(io.read())
  target_board = sel_list[sel]
  io.write("\n")
until target_board ~= nil

f = io.open("../NutConf.mk", "w")
f:write("# Automatically generated by nutsetup\n")
f:write("#\n")
f:write("# Do not edit, modify UserConf.mk instead!\n")
f:write("#\n\n")
f:write("PLATFORM="..target_board.name[2].."\n")
f:write("HWDEF+=-D$(PLATFORM)\n")
if target_device.arch == archs.avr then
  f:write("HWDEF+=-D__HARVARD_ARCH__\n")
end
f:write("ARCH="..get_arch_name(target_arch).."\n")
f:write("DEVICE="..string.upper(get_device_name(target_device)).."\n")
f:write("MCU="..get_device_name(target_device).."\n")
f:write("CRUROM=crurom\n")
f:write("\n\n")
f:write("include $(top_srcdir)/UserConf.mk\n")
f:close()

create_ifnotexists("../UserConf.mk", "#\n# You can use this file to modify values in NutConf.mk\n#\n")

