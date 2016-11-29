-------------------------------------------------------------------[27.11.2016]
-- Basic build 20161127
-- DEVBOARD DivGMX Rev.A
-------------------------------------------------------------------------------
-- Engineer: MVV <mvvproject@gmail.com>
--
-- https://github.com/mvvproject/DivGMX
--
-- Copyright (c) 2016 Vladislav Matlash
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- * Redistributions of source code must retain the above copyright notice,
--   this list of conditions and the following disclaimer.
--
-- * Redistributions in synthesized form must reproduce the above copyright
--   notice, this list of conditions and the following disclaimer in the
--   documentation and/or other materials provided with the distribution.
--
-- * Neither the name of the author nor the names of other contributors may
--   be used to endorse or promote products derived from this software without
--   specific prior written agreement from the author.
--
-- * License is granted for non-commercial use only.  A fee may not be charged
--   for redistributions as source code or in synthesized/hardware form without 
--   specific prior written agreement from the author.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

library IEEE; 
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all; 

entity basic is
port (
	-- Clock (50MHz)
	CLK_50MHZ	: in std_logic;
	-- HDMI
	TMDS		: out std_logic_vector(7 downto 0);
	-- USB Host (VNC2-32)
	USB_IO3		: in std_logic;
	USB_TXD		: in std_logic;
--	USB_RXD		: out std_logic;
	-- SPI (W25Q64/SD)
	DATA0		: in std_logic;
	ASDO		: out std_logic;
	DCLK		: out std_logic;
	NCSO		: out std_logic;
	-- I2C (HDMI/RTC)
--	I2C_SCL		: inout std_logic;
--	I2C_SDA		: inout std_logic;
	-- SD
	SD_NDET		: in std_logic;
	SD_NCS		: out std_logic;
	-- SDRAM (32M8)
	DRAM_DQ		: inout std_logic_vector(7 downto 0);
	DRAM_A		: out std_logic_vector(12 downto 0);
	DRAM_BA		: out std_logic_vector(1 downto 0);
	DRAM_DQM	: out std_logic;
	DRAM_CLK	: out std_logic;
	DRAM_NWE	: out std_logic;
	DRAM_NCAS	: out std_logic;
	DRAM_NRAS	: out std_logic;
	-- Audio
	OUT_L		: out std_logic;
	OUT_R		: out std_logic;
	-- ZXBUS
--	BUF_NINT	: in std_logic;
--	BUF_NNMI	: in std_logic;
	BUF_NRESET	: in std_logic;
	BUF_DIR		: out std_logic_vector(1 downto 0);
--	BUS_CLK		: inout std_logic;
	BUS_D		: inout std_logic_vector(7 downto 0);
	BUS_A		: inout std_logic_vector(15 downto 0);
	BUS_NMREQ	: inout std_logic;
	BUS_NIORQ	: inout std_logic;
--	BUS_NBUSACK	: inout std_logic;
	BUS_NRD		: inout std_logic;
	BUS_NWR		: inout std_logic;
	BUS_NM1		: inout std_logic;
	BUS_NRFSH	: inout std_logic;
	BUS_NINT	: out std_logic;
	BUS_NWAIT	: out std_logic;
	BUS_NBUSRQ	: out std_logic;
	BUS_NROMOE	: out std_logic;
	BUS_NIORQGE	: out std_logic);
end basic;

architecture rtl of basic is

--signal areset		: std_logic;
signal clk_vga		: std_logic;
signal clk_tmds		: std_logic;
signal clk_bus		: std_logic;
signal clk_sdr		: std_logic;

signal sync_hsync	: std_logic;
signal sync_vsync	: std_logic;
signal sync_blank	: std_logic;
signal sync_hcnt	: std_logic_vector(9 downto 0);
signal sync_vcnt	: std_logic_vector(9 downto 0);
signal sync_flash	: std_logic;

signal vga_r		: std_logic_vector(1 downto 0);
signal vga_g		: std_logic_vector(1 downto 0);
signal vga_b		: std_logic_vector(1 downto 0);
signal vga_di		: std_logic_vector(7 downto 0);
-- Audio
signal beeper		: std_logic_vector(7 downto 0);
signal audio_l		: std_logic_vector(15 downto 0);
signal audio_r		: std_logic_vector(15 downto 0);

signal vram_wr		: std_logic;

signal reg_mreq_n_i	: std_logic;
signal reg_iorq_n_i	: std_logic;
signal reg_rd_n_i	: std_logic;
signal reg_wr_n_i	: std_logic;
signal reg_a_i		: std_logic_vector(15 downto 0);
signal reg_d_i		: std_logic_vector(7 downto 0);
signal reg_reset_n_i	: std_logic;
signal reg_m1_n_i	: std_logic;
signal reg_rfsh_n_i	: std_logic;
signal mreq_n_i		: std_logic;
signal iorq_n_i		: std_logic;
signal rd_n_i		: std_logic;
signal wr_n_i		: std_logic;
signal a_i		: std_logic_vector(15 downto 0);
signal d_i		: std_logic_vector(7 downto 0);
signal reset_n_i	: std_logic;
signal m1_n_i		: std_logic;
signal rfsh_n_i		: std_logic;

signal vram_scr		: std_logic;
signal ram_addr		: std_logic_vector(7 downto 0);
signal port_7ffd_reg	: std_logic_vector(7 downto 0) := "00010000";
signal mux		: std_logic_vector(3 downto 0);
signal port_xxfe_reg	: std_logic_vector(7 downto 0);
signal vga_addr		: std_logic_vector(12 downto 0);
signal rom_do		: std_logic_vector(7 downto 0);
-- DIVMMC
signal divmmc_do	: std_logic_vector(7 downto 0);
signal divmmc_amap	: std_logic;
signal divmmc_e3reg	: std_logic_vector(7 downto 0);	
signal divmmc_ncs	: std_logic;
signal divmmc_sclk	: std_logic;
signal divmmc_mosi	: std_logic;
-- SDRAM
signal sdr_do_bus	: std_logic_vector(7 downto 0);
signal sdr_wr		: std_logic;
signal sdr_rd		: std_logic;
signal sdr_rfsh		: std_logic;

signal selector		: std_logic_vector(3 downto 0);

-- Soundrive
signal covox_a		: std_logic_vector(7 downto 0);
signal covox_b		: std_logic_vector(7 downto 0);
signal covox_c		: std_logic_vector(7 downto 0);
signal covox_d		: std_logic_vector(7 downto 0);
-- TurboSound
signal ssg_sel		: std_logic;
signal ssg0_do_bus	: std_logic_vector(7 downto 0);
signal ssg0_a		: std_logic_vector(7 downto 0);
signal ssg0_b		: std_logic_vector(7 downto 0);
signal ssg0_c		: std_logic_vector(7 downto 0);
signal ssg1_do_bus	: std_logic_vector(7 downto 0);
signal ssg1_a		: std_logic_vector(7 downto 0);
signal ssg1_b		: std_logic_vector(7 downto 0);
signal ssg1_c		: std_logic_vector(7 downto 0);
-- Z-Controller
signal zc_do_bus	: std_logic_vector(7 downto 0);
signal zc_ncs		: std_logic;
signal zc_sclk		: std_logic;
signal zc_mosi		: std_logic;
signal zc_rd		: std_logic;
signal zc_wr		: std_logic;
-- Mouse
signal ms_x		: std_logic_vector(7 downto 0);
signal ms_y		: std_logic_vector(7 downto 0);
signal ms_z		: std_logic_vector(7 downto 0);
signal ms_b		: std_logic_vector(7 downto 0);
-- Keyboard
signal kb_do_bus	: std_logic_vector(4 downto 0);
signal kb_fn_bus	: std_logic_vector(12 downto 1);
signal kb_fn		: std_logic_vector(12 downto 1);
signal key		: std_logic_vector(12 downto 1) := "000000000000";

signal ena_1_75mhz	: std_logic;
signal ena_cnt		: std_logic_vector(5 downto 0);

--Вывод изображения на HDMI со звуком +
--DivMMC/Z-Controller +
--CMOS (стандарт Mr. Gluk)
--Kempston joystick/Gamepad
--Kempston mouse +
--SounDrive +
--Turbo Sound +
--Чтение порта #7FFD +

begin

-- PLL
U1: entity work.altpll0
port map (
	areset		=> '0',
	locked		=> open,
	inclk0		=> CLK_50MHZ,	--  50.00 MHz
	c0		=> clk_vga,	--  25.20 MHz
	c1		=> clk_tmds,	-- 126.00 MHz
	c2		=> clk_bus,	--  28.00 MHz
	c3		=> clk_sdr);	--  84.00 MHz

-- HDMI
U2: entity work.hdmi
generic map (
	FREQ		=> 25200000,	-- pixel clock frequency = 25.2MHz
	FS		=> 48000,	-- audio sample rate - should be 32000, 41000 or 48000 = 48KHz
	CTS		=> 25200,	-- CTS = Freq(pixclk) * N / (128 * Fs)
	N		=> 6144)	-- N = 128 * Fs /1000,  128 * Fs /1500 <= N <= 128 * Fs /300 (Check HDMI spec 7.2 for details)
port map (
	I_CLK_VGA	=> clk_vga,
	I_CLK_TMDS	=> clk_tmds,
	I_HSYNC		=> sync_hsync,
	I_VSYNC		=> sync_vsync,
	I_BLANK		=> sync_blank,
	I_RED		=> vga_r & vga_r & vga_r & vga_r,
	I_GREEN		=> vga_g & vga_g & vga_g & vga_g,
	I_BLUE		=> vga_b & vga_b & vga_b & vga_b,
	I_AUDIO_PCM_L 	=> audio_l,
	I_AUDIO_PCM_R	=> audio_r,
	O_TMDS		=> TMDS);

-- Sync
U3: entity work.sync
port map (
	I_CLK		=> clk_vga,
	I_EN		=> '1',
	O_HCNT		=> sync_hcnt,
	O_VCNT		=> sync_vcnt,
	O_INT		=> open,
	O_FLASH		=> sync_flash,
	O_BLANK		=> sync_blank,
	O_HSYNC		=> sync_hsync,
	O_VSYNC		=> sync_vsync);	

-- Video
U4: entity work.vga_zx
port map (
	I_CLK		=> clk_vga,
	I_CLKEN		=> '1',
	I_DATA		=> vga_di,
	I_BORDER	=> port_xxfe_reg(2 downto 0),	-- Биты D0..D2 порта xxFE определяют цвет бордюра
	I_HCNT		=> sync_hcnt,
	I_VCNT		=> sync_vcnt,
	I_BLANK		=> sync_blank,
	I_FLASH		=> sync_flash,
	O_ADDR		=> vga_addr,
	O_PAPER		=> open,
	O_RED		=> vga_r,
	O_GREEN		=> vga_g,
	O_BLUE		=> vga_b);	
	
-- Video RAM 16K
U5: entity work.vram
port map (
	address_a	=> vram_scr & a_i(12 downto 0),
	address_b	=> port_7ffd_reg(3) & vga_addr,
	clock_a		=> clk_bus,
	clock_b		=> clk_vga,
	data_a	 	=> d_i,
	data_b	 	=> (others => '0'),
	wren_a	 	=> vram_wr,
	wren_b	 	=> '0',
	q_a	 	=> open,
	q_b	 	=> vga_di);

U6: entity work.rom
port map (
	address		=> a_i(12 downto 0),
	clock		=> clk_bus,
	q		=> rom_do);

-- SDRAM Controller
U7: entity work.sdram
port map (
	CLK		=> clk_sdr,
	A		=> "0000" & ram_addr & a_i(12 downto 0),
	DI		=> d_i,
	DO		=> sdr_do_bus,
	DM	 	=> '0',
	WR		=> sdr_wr,
	RD		=> sdr_rd,
	RFSH		=> sdr_rfsh,
	RFSHREQ		=> open,
	IDLE		=> open,
	CK		=> DRAM_CLK,
	CKE		=> open,
	RAS_n		=> DRAM_NRAS,
	CAS_n		=> DRAM_NCAS,
	WE_n		=> DRAM_NWE,
	DQM		=> DRAM_DQM,
	BA1		=> DRAM_BA(1),
	BA0		=> DRAM_BA(0),
	MA		=> DRAM_A,
	DQ		=> DRAM_DQ);	
	
-- DIVMMC Interface
U8: entity work.divmmc
port map (
	CLK		=> clk_bus,
	EN		=> kb_fn(6),
	RESET_N		=> reset_n_i,
	ADDR		=> a_i,
	DI		=> d_i,
	DO		=> divmmc_do,
	WR_N		=> wr_n_i,
	RD_N		=> rd_n_i,
	IORQ_N		=> iorq_n_i,
	MREQ_N		=> mreq_n_i,
	M1_N		=> m1_n_i,
	E3REG		=> divmmc_e3reg,
	AMAP		=> divmmc_amap,
	CS_N		=> divmmc_ncs,
	SCLK		=> divmmc_sclk,
	MOSI		=> divmmc_mosi,
	MISO		=> DATA0);	

-- Soundrive
U9: entity work.soundrive
port map (
	I_RESET		=> not reset_n_i,
	I_CLK		=> clk_bus,
	I_CS		=> '1',
	I_WR_N		=> wr_n_i,
	I_ADDR		=> a_i(7 downto 0),
	I_DATA		=> d_i,
	I_IORQ_N	=> iorq_n_i,
	I_DOS		=> '0',
	O_COVOX_A	=> covox_a,
	O_COVOX_B	=> covox_b,
	O_COVOX_C	=> covox_c,
	O_COVOX_D	=> covox_d);
	
-- TurboSound
U10: entity work.turbosound
port map (
	I_CLK		=> clk_bus,
	I_ENA		=> ena_1_75mhz,
	I_ADDR		=> a_i,
	I_DATA		=> d_i,
	I_WR_N		=> wr_n_i,
	I_IORQ_N	=> iorq_n_i,
	I_M1_N		=> m1_n_i,
	I_RESET_N	=> reset_n_i,
	O_SEL		=> ssg_sel,
	-- ssg0
	I_SSG0_IOA	=> (others => '1'),
	O_SSG0_IOA	=> open,
	I_SSG0_IOB	=> (others => '1'),
	O_SSG0_IOB	=> open,
	O_SSG0_DA	=> ssg0_do_bus,
	O_SSG0_AUDIO	=> open,
	O_SSG0_AUDIO_A	=> ssg0_a,
	O_SSG0_AUDIO_B	=> ssg0_b,
	O_SSG0_AUDIO_C	=> ssg0_c,
	-- ssg1
	I_SSG1_IOA	=> (others => '1'),
	O_SSG1_IOA	=> open,
	I_SSG1_IOB	=> (others => '1'),
	O_SSG1_IOB	=> open,
	O_SSG1_DA	=> ssg1_do_bus,
	O_SSG1_AUDIO	=> open,
	O_SSG1_AUDIO_A	=> ssg1_a,
	O_SSG1_AUDIO_B	=> ssg1_b,
	O_SSG1_AUDIO_C	=> ssg1_c);
	
-- Z-Controller
U11: entity work.zcontroller
port map (
	I_RESET		=> not reset_n_i,
	I_CLK		=> clk_bus,
	I_ADDR		=> a_i(5),
	I_DATA		=> d_i,
	O_DATA		=> zc_do_bus,
	I_RD		=> zc_rd,
	I_WR		=> zc_wr,
	I_SDDET		=> SD_NDET,
	I_SDPROT	=> '0',
	O_CS_N		=> zc_ncs,
	O_SCLK		=> zc_sclk,
	O_MOSI		=> zc_mosi,
	I_MISO		=> DATA0);

-- Delta-Sigma
U12: entity work.dac
generic map (
	msbi_g		=> 15)
port map (
	I_CLK		=> clk_sdr,
	I_RESET		=> not reset_n_i,
	I_DATA		=> audio_l,
	O_DAC		=> OUT_L);

U13: entity work.dac
generic map (
	msbi_g		=> 15)
port map (
	I_CLK		=> clk_sdr,
	I_RESET		=> not reset_n_i,
	I_DATA		=> audio_r,
	O_DAC		=> OUT_R);	

-- USB HID
U14: entity work.deserializer
generic map (
	divisor			=> 434)		-- divisor = 50MHz / 115200 Baud = 434
port map(
	I_CLK			=> CLK_50MHZ,
	I_RESET			=> not reset_n_i,
	I_RX			=> USB_TXD,
	I_NEWFRAME		=> USB_IO3,
	I_ADDR			=> a_i(15 downto 8),
	O_MOUSE_X		=> ms_x,
	O_MOUSE_Y		=> ms_y,
	O_MOUSE_Z		=> ms_z,
	O_MOUSE_BUTTONS		=> ms_b,
	O_KEY0			=> open,--kb_key0,
	O_KEY1			=> open,--kb_key1,
	O_KEY2			=> open,--kb_key2,
	O_KEY3			=> open,--kb_key3,
	O_KEY4			=> open,--kb_key4,
	O_KEY5			=> open,--kb_key5,
	O_KEY6			=> open,--kb_key6,
	O_KEYBOARD_SCAN		=> kb_do_bus,
	O_KEYBOARD_FKEYS	=> kb_fn_bus,
	O_KEYBOARD_JOYKEYS	=> open,--kb_joy_bus,
	O_KEYBOARD_CTLKEYS	=> open);--kb_soft_bus);

-------------------------------------------------------------------------------	
-- F6 = Z-Controller/DivMMC
process (clk_bus, key, kb_fn_bus, kb_fn)
begin
	if (clk_bus'event and clk_bus = '1') then
		key <= kb_fn_bus;
		if (kb_fn_bus /= key) then
			kb_fn <= kb_fn xor key;
		end if;
	end if;
end process;

-------------------------------------------------------------------------------	
-- Z-Controller	
zc_wr 	<= '1' when (iorq_n_i = '0' and wr_n_i = '0' and a_i(7 downto 6) = "01" and a_i(4 downto 0) = "10111") else '0';
zc_rd 	<= '1' when (iorq_n_i = '0' and rd_n_i = '0' and a_i(7 downto 6) = "01" and a_i(4 downto 0) = "10111") else '0';
	
-------------------------------------------------------------------------------
-- SDRAM
sdr_wr <= '1' when mreq_n_i = '0' and wr_n_i = '0' and mux = "1001" else '0';
sdr_rd <= not (mreq_n_i or rd_n_i);
sdr_rfsh <= not rfsh_n_i;

-- Clock
process (clk_bus)
begin
	if clk_bus' event and clk_bus = '0' then
		ena_cnt <= ena_cnt + 1;
	end if;
end process;

ena_1_75mhz <= ena_cnt(3) and ena_cnt(2) and ena_cnt(1) and ena_cnt(0);
	
--areset <= not reset_n_i;	-- глобальный сброс

-------------------------------------------------------------------------------
-- Video
vram_scr <= '1' when (ram_addr = "00001110") else '0';
vram_wr  <= '1' when (mreq_n_i = '0' and wr_n_i = '0' and ((ram_addr = "00001010") or (ram_addr = "00001110"))) else '0';

------------------------------------------------------------------------------
-- Селектор
mux <= ((divmmc_amap or divmmc_e3reg(7)) and kb_fn(6)) & a_i(15 downto 13);

process (mux, port_7ffd_reg, ram_addr, divmmc_e3reg)
begin
	case mux is
		when "0000"|"0001" => ram_addr <= "10000001";
		when "1000" => ram_addr <= "10000000";						-- ESXDOS ROM 0000-1FFF
		when "1001" => ram_addr <= "01" & divmmc_e3reg(5 downto 0);			-- ESXDOS RAM 2000-3FFF
	
		when "0010"|"1010" => ram_addr <= "00001010";					-- Seg1 RAM 4000-5FFF
		when "0011"|"1011" => ram_addr <= "00001011";					-- Seg1 RAM 6000-7FFF
		when "0100"|"1100" => ram_addr <= "00000100";					-- Seg2 RAM 8000-9FFF
		when "0101"|"1101" => ram_addr <= "00000101";					-- Seg2 RAM A000-BFFF
		when "0110"|"1110" => ram_addr <= "0000" & port_7ffd_reg(2 downto 0) & '0';	-- Seg3 RAM C000-DFFF
		when "0111"|"1111" => ram_addr <= "0000" & port_7ffd_reg(2 downto 0) & '1';	-- Seg3 RAM E000-FFFF
		when others => null;
	end case;
end process;

-------------------------------------------------------------------------------
-- SD DIVMMC/Z-Controller/SPIFLASH
SD_NCS	<= divmmc_ncs when kb_fn(6) = '1' else zc_ncs;
DCLK	<= divmmc_sclk when kb_fn(6) = '1' else zc_sclk;
ASDO	<= divmmc_mosi when kb_fn(6) = '1' else zc_mosi;
NCSO	<= '1';

-------------------------------------------------------------------------------
-- Регистры
process (reset_n_i, clk_bus, a_i, port_7ffd_reg, wr_n_i, d_i, iorq_n_i)
begin
	if (reset_n_i = '0') then
		port_7ffd_reg <= "00010000";
	elsif (clk_bus'event and clk_bus = '1') then
		if (iorq_n_i = '0' and wr_n_i = '0' and a_i = X"7FFD" and port_7ffd_reg(5) = '0') then port_7ffd_reg <= d_i; end if;	-- D7-D6:не используются; D5:1=запрещение расширенной памяти (48K защёлка); D4=номер страницы ПЗУ(0-BASIC128, 1-BASIC48); D3=выбор отображаемой видеостраницы(0-страница в банке 5, 1 - в банке 7); D2-D0=номер страницы ОЗУ подключенной в верхние 16 КБ памяти (с адреса #C000)
	end if;
end process;

process (clk_bus, a_i, port_xxfe_reg, wr_n_i, d_i, iorq_n_i)
begin
	if (clk_bus'event and clk_bus = '1') then                  
		if (iorq_n_i = '0' and wr_n_i = '0' and a_i(7 downto 0) = X"FE") then port_xxfe_reg <= d_i; end if;	-- D7-D5=не используются; D4=бипер; D3=MIC; D2-D0=цвет бордюра
	end if;
end process;

selector <=	X"0" when (mreq_n_i = '0' and rd_n_i = '0' and ram_addr = "10000000") else
		X"1" when (mreq_n_i = '0' and rd_n_i = '0' and ram_addr(7 downto 6) = "01") else
		X"2" when (iorq_n_i = '0' and rd_n_i = '0' and a_i(7 downto 0) = X"EB" and kb_fn(6) = '1') else			-- DivMMC
		X"3" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"FFFD" and ssg_sel = '0') else				-- TurboSound
		X"4" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"FFFD" and ssg_sel = '1') else				-- TurboSound
		X"5" when (iorq_n_i = '0' and rd_n_i = '0' and a_i(7 downto 6) = "01" and a_i(4 downto 0) = "10111" and kb_fn(6) = '0') else 	-- Z-Controller
		X"6" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"FADF") else						-- Mouse key
		X"7" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"FBDF") else						-- Mouse x
		X"8" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"FFDF") else						-- Mouse y
		X"9" when (iorq_n_i = '0' and rd_n_i = '0' and a_i = X"7FFD") else						-- Read port #7FFD
		X"A" when (iorq_n_i = '0' and rd_n_i = '0' and a_i(7 downto 0) = X"FE") else					-- Read port #xxFE Keyboard
		(others => '1');

-------------------------------------------------------------------------------
-- Audio
beeper	<= (others => port_xxfe_reg(4));
audio_l	<= ("000" & beeper & "00000") + ("000" & ssg0_a & "00000") + ("000" & ssg0_b & "00000") + ("000" & ssg1_a & "00000") + ("000" & ssg1_b & "00000") + ("000" & covox_a & "00000") + ("000" & covox_b & "00000");
audio_r	<= ("000" & beeper & "00000") + ("000" & ssg0_c & "00000") + ("000" & ssg0_b & "00000") + ("000" & ssg1_c & "00000") + ("000" & ssg1_b & "00000") + ("000" & covox_c & "00000") + ("000" & covox_d & "00000");

-------------------------------------------------------------------------------
-- ZX-BUS
BUS_NINT	<= '1';
BUS_NWAIT	<= '1';
BUS_NBUSRQ	<= '1';
BUS_NROMOE	<= '1' when selector = X"0" or selector = X"1" else '0';
BUS_NIORQGE	<= '0' when selector = X"0" or selector = X"1" or selector = X"A" or selector = X"F" else '1';	-- Read system port #xxFE On
BUF_DIR		<= "00" when selector = X"F" else "10";
BUS_A		<= (others => 'Z');
BUS_NMREQ	<= 'Z';
BUS_NIORQ	<= 'Z';
BUS_NRD		<= 'Z';
BUS_NWR		<= 'Z';
BUS_NM1		<= 'Z';
BUS_NRFSH	<= 'Z';

process (selector, rom_do, divmmc_do, sdr_do_bus, ssg0_do_bus, ssg1_do_bus, zc_do_bus, ms_z, ms_b, ms_x, ms_y, port_7ffd_reg, kb_do_bus)
begin
	case selector is
		when X"0" => BUS_D <= rom_do;
		when X"1" => BUS_D <= sdr_do_bus;
		when X"2" => BUS_D <= divmmc_do;
		when X"3" => BUS_D <= ssg0_do_bus;
		when X"4" => BUS_D <= ssg1_do_bus;
		when X"5" => BUS_D <= zc_do_bus;
		when X"6" => BUS_D <= ms_z(3 downto 0) & '1' & not ms_b(2 downto 0);
		when X"7" => BUS_D <= ms_x;
		when X"8" => BUS_D <= not ms_y;
		when X"9" => BUS_D <= port_7ffd_reg;
		when X"A" => BUS_D <= "111" & kb_do_bus;
		when others => BUS_D <= (others => 'Z');
	end case;
end process;
		
process (clk_bus)
begin
	if clk_bus'event and clk_bus = '1' then
		reg_mreq_n_i	<= BUS_NMREQ;
		reg_iorq_n_i	<= BUS_NIORQ;
		reg_rd_n_i	<= BUS_NRD;
		reg_wr_n_i	<= BUS_NWR;
		reg_a_i		<= BUS_A;
		reg_d_i		<= BUS_D;
		reg_reset_n_i	<= BUF_NRESET;
		reg_m1_n_i	<= BUS_NM1;
		reg_rfsh_n_i	<= BUS_NRFSH;
		
		mreq_n_i	<= reg_mreq_n_i;
		iorq_n_i	<= reg_iorq_n_i;
		rd_n_i		<= reg_rd_n_i;
		wr_n_i		<= reg_wr_n_i;
		a_i		<= reg_a_i;
		d_i		<= reg_d_i;
		reset_n_i	<= reg_reset_n_i;
		m1_n_i		<= reg_m1_n_i;
		rfsh_n_i	<= reg_rfsh_n_i;
	end if;
end process;
		
		
		

end rtl;
