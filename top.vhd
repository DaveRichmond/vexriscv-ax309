library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library UNISIM;
use UNISIM.vcomponents.ALL;
library work;
use work.all;

entity AX309 is
	generic
	(
		debug_enable : integer := 0
	);
	port 
	(
		--//////////// CLOCK //////////
		CLK_50M : in std_logic;

		--//////////// LED //////////
		LED : out std_logic_vector(3 downto 0) := (others => '0');

		--//////////// KEY //////////
		KEY : in std_logic_vector(3 downto 0);
		

		
		--///////// VGA /////////
		VGA_HS : out std_logic := '1';
		VGA_VS : out std_logic := '1';
		VGA_RED : out std_logic_vector(4 downto 0) := (others => '1');
		VGA_GREEN : out std_logic_vector(5 downto 0) := (others => '1');
		VGA_BLUE : out std_logic_vector(4 downto 0) := (others => '1');
		
		-- /////// UART ///////
		UART_TX : out std_logic := '1';
		UART_RX : in std_logic;

		--//////////// SDRAM //////////
		SDRAM_A : out std_logic_vector(12 downto 0) := (others => '1');
		SDRAM_BA : out std_logic_vector(1 downto 0) := (others => '1');
		SDRAM_NCAS : out std_logic := '1';
		SDRAM_CKE : out std_logic := '1';
		SDRAM_CLK : out std_logic := '1';
		SDRAM_NCS : out std_logic := '1';
		SDRAM_DB : inout std_logic_vector(15 downto 0) := (others => 'Z');
		SDRAM_DQM : inout std_logic_vector(1 downto 0) := "ZZ";
		SDRAM_NRAS : out std_logic := '1';
		SDRAM_NWE : out std_logic := '1';

		--//////////// EEPROM //////////
		I2C_SCL : out std_logic  := '1';
		I2C_SDA : inout std_logic := 'Z';
		
		-- //////// SD CARD ////////
		SD_CLK : out std_logic := '1';
		SD_CS : out std_logic := '1';
		SD_DataIn : out std_logic := '1'; -- in of the card, out for us
		SD_DataOut : in std_logic; -- out of the card, in for us
		
		-- //////// Camera ////////
		CAMERA_RST_N : out std_logic := '1';
		CAMERA_PWDN : out std_logic := '1';
		CAMERA_XCLK : in std_logic;
		CAMERA_PCLK : in std_logic;
		CAMERA_HREF : in std_logic;
		CAMERA_VSYNC : in std_logic;
		CAMERA_DB : inout std_logic_vector(7 downto 0) := (others => 'Z');
		CAMERA_SCLK : out std_logic := '1';
		CAMERA_SDAT : inout std_logic := 'Z';
		
		-- note: typo in ucf for following, missing several io pins
		--//////////// 2x20 GPIO Header(2x3 reserved for power) //////////
		GPIO_0_D : inout std_logic_vector(33 downto 0) := (others => 'Z');

		--//////////// 2x20 GPIO Header (2x3 reserved for power) //////////
		GPIO_1_D : inout std_logic_vector(33 downto 0) := (others => 'Z')
                             
	);
end entity AX309;

architecture rtl of AX309 is
	signal clk_core : std_logic;
	signal clk_sdram : std_logic;
	signal clk_25M : std_logic;
	
	signal gpioA_read : std_logic_vector(31 downto 0);
	signal gpioA_write : std_logic_vector(31 downto 0);
	signal gpioA_writeEnable  : std_logic_vector(31 downto 0);
	
	signal gpioB_read : std_logic_vector(31 downto 0);
	signal gpioB_write : std_logic_vector(31 downto 0);
	signal gpioB_writeEnable  : std_logic_vector(31 downto 0);

	--//////// JTAG ////////
	signal JTAG_TCK 	: std_logic;
	signal JTAG_TMS 	: std_logic;
	signal JTAG_TDO 	: std_logic;
	signal JTAG_TDI 	: std_logic;
  
   signal io_sdram_DQ_write : std_logic_vector(15 downto 0);
   signal io_sdram_DQ_writeEnable : std_logic;
	
	component pll is
		PORT
		(
			CLK_IN1		: IN STD_LOGIC  := '0';
			CLK_OUT1		: OUT STD_LOGIC ;
			CLK_OUT2		: OUT STD_LOGIC ;
			CLK_OUT3		: OUT STD_LOGIC 
		);
	end component pll;
	
	component Briey is
		port (
      io_asyncReset :in std_logic;
      io_axiClk    : in std_logic;
      io_vgaClk    : in std_logic;
      io_jtag_tck : in std_logic;
      io_jtag_tms : in std_logic;
      io_jtag_tdi : in std_logic;
      io_jtag_tdo : out std_logic;
      io_gpioA_read : in std_logic_vector(31 downto 0);
      io_gpioA_write : out std_logic_vector(31 downto 0);
      io_gpioA_writeEnable : out std_logic_vector(31 downto 0);
      io_gpioB_read : in std_logic_vector(31 downto 0);
      io_gpioB_write : out std_logic_vector(31 downto 0);
      io_gpioB_writeEnable : out std_logic_vector(31 downto 0);
      io_timerExternal_clear  : in std_logic;
		io_timerExternal_tick  : in std_logic;
		io_coreInterrupt  : in std_logic;
      io_uart_txd :  out std_logic;
      io_uart_rxd    : in std_logic;
      io_sdram_ADDR : out std_logic_vector(12 downto 0);
      io_sdram_BA : out std_logic_vector(1 downto 0);
      io_sdram_DQ_read : in std_logic_vector(15 downto 0);
      io_sdram_DQ_write : out std_logic_vector(15 downto 0);
      io_sdram_DQ_writeEnable : out std_logic;
      io_sdram_DQM : out std_logic_vector(1 downto 0);
      io_sdram_CASn : out std_logic;
      io_sdram_CKE : out std_logic;
      io_sdram_CSn : out std_logic;
      io_sdram_RASn : out std_logic;
      io_sdram_WEn : out std_logic ;
	   io_vga_vSync : out std_logic;
      io_vga_hSync : out std_logic;
      io_vga_colorEn : out std_logic;
      io_vga_color_r : out std_logic_vector(4 downto 0);
      io_vga_color_g : out std_logic_vector(5 downto 0);
      io_vga_color_b : out std_logic_vector(4 downto 0) 
		);
	end component;	

begin

	pll_inst : pll 
		PORT MAP (
			CLK_IN1	 => CLK_50M,
			CLK_OUT1 => clk_core,
			CLK_OUT2 => clk_sdram,
			CLK_OUT3 => clk_25M
		);
  SDRAM_CLK <= clk_sdram;

	-- can we use the bscan interface? Unable to get it to work right now so let's instead map to the last 4 GPIOs
	--BSCAN_SPARTAN6_inst : BSCAN_SPARTAN6
	--generic map (
	--	JTAG_CHAIN => 1 -- chain number
	--) 
	--port map (
	--	TCK => JTAG_TCK,
	--	TMS => JTAG_TMS,
	--	TDO => JTAG_TDO,
	--	TDI => JTAG_TDI
	--);
	JTAG_TCK <= GPIO_1_D(33);
	JTAG_TMS <= GPIO_1_D(32);
	JTAG_TDO <= GPIO_1_D(31);
	JTAG_TDI <= GPIO_1_D(30);
	
  LED(3 downto 0) <=   gpioA_write(3 downto 0);   -- mirror GPIOA(3:0) onto LEDs
  --gpioA_read(3 downto 0) <= SW(3 downto 0);
  --gpioA_read(31 downto 0) <= GPIO_0_D(31 downto 0);
  --GPIO_0_D(31 downto 0) <= gpioA_write(31 downto 0);
  --gpioB_read(31 downto 0) <= GPIO_1_D(31 downto 0);
  --GPIO_1_D(31 downto 0) <= gpioB_write(31 downto 0);
 
  briey_inst : Briey
    port map (
      io_asyncReset =>  not KEY(0),
      io_axiClk   =>  clk_core,
      io_vgaClk   =>  clk_25M,
      io_jtag_tck =>  JTAG_TCK,
      io_jtag_tms =>  JTAG_TMS,
      io_jtag_tdi =>  JTAG_TDI,
      io_jtag_tdo =>  JTAG_TDO,
		-- work out how to connect jtag later
		--io_jtag_tck => '1',
		--io_jtag_tms => '1',
		--io_jtag_tdi => '1',
		--io_jtag_tdo => open,
      io_gpioA_read =>  gpioA_read,
      io_gpioA_write =>  gpioA_write,
      io_gpioA_writeEnable =>  gpioA_writeEnable,
      io_gpioB_read =>  gpioB_read,
      io_gpioB_write =>  gpioB_write,
      io_gpioB_writeEnable =>  gpioB_writeEnable,
	   io_timerExternal_clear => '0',
	   io_timerExternal_tick => '0',
	   io_uart_txd => UART_TX,
	   io_uart_rxd => UART_RX,
		io_coreInterrupt => not KEY(1),
      io_sdram_ADDR => SDRAM_A,
      io_sdram_BA => SDRAM_BA,
      io_sdram_DQ_read => SDRAM_DB,
      io_sdram_DQ_write => io_sdram_DQ_write,
      io_sdram_DQ_writeEnable => io_sdram_DQ_writeEnable,
      io_sdram_DQM => SDRAM_DQM,
      io_sdram_CASn => SDRAM_NCAS,
      io_sdram_CKE  => SDRAM_CKE,
      io_sdram_CSn  => SDRAM_NCS,
      io_sdram_RASn  => SDRAM_NRAS,
      io_sdram_WEn  => SDRAM_NWE,
	   io_vga_vSync => VGA_VS,
      io_vga_hSync => VGA_HS,
      io_vga_color_r => VGA_RED,
      io_vga_color_g => VGA_GREEN,
      io_vga_color_b => VGA_BLUE
    );	
	 
	 
    SDRAM_DB <= io_sdram_DQ_write when io_sdram_DQ_writeEnable = '1' else (others => 'Z');
end architecture rtl; -- of xf