----------------------------------------------------------------------------------
-- Company:  ZITI
-- Engineer:  wgao
-- 
-- Create Date:    16:37:22 12 Feb 2009
-- Design Name: 
-- Module Name:    eb_wrapper - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

library work;
use work.abb64Package.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity eb_wrapper is
    Generic (
             C_ASYNFIFO_WIDTH  :  integer  :=  72
            );
    Port ( 
          
			 --FIFO PCIe-->USER
			 H2B_wr_clk        : IN  std_logic;
          H2B_wr_en         : IN  std_logic;
          H2B_wr_din        : IN  std_logic_VECTOR(C_ASYNFIFO_WIDTH-1 downto 0);
          H2B_wr_pfull      : OUT std_logic;
          H2B_wr_full       : OUT std_logic;
          H2B_wr_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
          H2B_rd_clk        : IN  std_logic;
          H2B_rd_en         : IN  std_logic;
          H2B_rd_dout       : OUT std_logic_VECTOR(C_ASYNFIFO_WIDTH-1 downto 0);
          H2B_rd_pempty     : OUT std_logic;
          H2B_rd_empty      : OUT std_logic;
          H2B_rd_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
			 H2B_rd_valid      : OUT std_logic;
			 --FIFO USER-->PCIe
          B2H_wr_clk        : IN  std_logic;
          B2H_wr_en         : IN  std_logic;
          B2H_wr_din        : IN  std_logic_VECTOR(C_ASYNFIFO_WIDTH-1 downto 0);
          B2H_wr_pfull      : OUT std_logic;
          B2H_wr_full       : OUT std_logic;
          B2H_wr_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
          B2H_rd_clk        : IN  std_logic;
          B2H_rd_en         : IN  std_logic;
          B2H_rd_dout       : OUT std_logic_VECTOR(C_ASYNFIFO_WIDTH-1 downto 0);
          B2H_rd_pempty     : OUT std_logic;
          B2H_rd_empty      : OUT std_logic;
          B2H_rd_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
			 B2H_rd_valid		 : OUT std_logic;	
          --RESET from PCIe
			 rst               : IN  std_logic
          );
end entity eb_wrapper;


architecture Behavioral of eb_wrapper is

  ---  32768 x 64, with data count synchronized to rd_clk
  component v6_eb_fifo_counted_resized
    port (
      wr_clk        : IN  std_logic;
      wr_en         : IN  std_logic;
      din           : IN  std_logic_VECTOR(C_ASYNFIFO_WIDTH-1-8 downto 0);
      prog_full     : OUT std_logic;
      full          : OUT std_logic;

      rd_clk        : IN  std_logic;
      rd_en         : IN  std_logic;
      dout          : OUT std_logic_VECTOR(C_ASYNFIFO_WIDTH-1-8 downto 0);
      prog_empty    : OUT std_logic;
      empty         : OUT std_logic;
      rd_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
      wr_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
		valid         : OUT std_logic; 
      rst           : IN  std_logic
      );
  end component;

  signal B2H_rd_data_count_wire    : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal B2H_rd_data_count_i       : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal H2B_rd_data_count_wire    : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal H2B_rd_data_count_i       : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal B2H_wr_data_count_wire    : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal B2H_wr_data_count_i       : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal H2B_wr_data_count_wire    : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
  signal H2B_wr_data_count_i       : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);


  signal resized_H2B_wr_din  : std_logic_VECTOR(64-1 downto 0);
  signal resized_H2B_rd_dout : std_logic_VECTOR(64-1 downto 0);
  signal resized_B2H_wr_din  : std_logic_VECTOR(64-1 downto 0);
  signal resized_B2H_rd_dout : std_logic_VECTOR(64-1 downto 0);


begin

  B2H_rd_data_count      <= B2H_rd_data_count_i;
  H2B_rd_data_count      <= H2B_rd_data_count_i;
  B2H_wr_data_count      <= B2H_wr_data_count_i;
  H2B_wr_data_count      <= H2B_wr_data_count_i;

  resized_H2B_wr_din  <= H2B_wr_din(64-1 downto 0);	
  resized_B2H_wr_din  <= B2H_wr_din(64-1 downto 0);	
  

  H2B_rd_dout(71 downto 64) <= C_ALL_ZEROS(71 downto 64);
  H2B_rd_dout(63 downto  0) <= resized_H2B_rd_dout;	

  B2H_rd_dout(71 downto 64) <= C_ALL_ZEROS(71 downto 64);
  B2H_rd_dout(63 downto  0) <= resized_B2H_rd_dout;	




  --  ------------------------------------------
  Syn_B2H_rd_data_count:
  process (B2H_rd_clk)
  begin
    if B2H_rd_clk'event and B2H_rd_clk = '1' then
       B2H_rd_data_count_i    <= B2H_rd_data_count_wire;
    end if;
  end process;

  Syn_H2B_rd_data_count:
  process (H2B_rd_clk)
  begin
    if H2B_rd_clk'event and H2B_rd_clk = '1' then
       H2B_rd_data_count_i    <= H2B_rd_data_count_wire;
    end if;
  end process;

  Syn_H2B_wr_data_count:
  process (H2B_wr_clk)
  begin
    if H2B_wr_clk'event and H2B_wr_clk = '1' then
       H2B_wr_data_count_i    <= H2B_wr_data_count_wire;
    end if;
  end process;

  Syn_B2H_wr_data_count:
  process (B2H_wr_clk)
  begin
    if B2H_wr_clk'event and B2H_wr_clk = '1' then
       B2H_wr_data_count_i    <= B2H_wr_data_count_wire;
    end if;
  end process;
  --  ------------------------------------------


----- Host2Board FIFO ----------
  U0_H2B:
  v6_eb_fifo_counted_resized
    port map (
         wr_clk         => H2B_wr_clk      			,
         wr_en          => H2B_wr_en       			,
         din            => resized_H2B_wr_din 		,	
         prog_full      => H2B_wr_pfull    			,
         full           => H2B_wr_full     			,
         rd_clk         => H2B_rd_clk              ,
         rd_en          => H2B_rd_en               ,
         dout           => resized_H2B_rd_dout     ,
         prog_empty     => H2B_rd_pempty           ,
         empty          => H2B_rd_empty            ,
         rd_data_count  => H2B_rd_data_count_wire  ,
         wr_data_count  => H2B_wr_data_count_wire  ,
			valid          => H2B_rd_valid   		   ,
         rst        		=> rst      
         );


----- Board2Host FIFO ----------
  U0_B2H:
  v6_eb_fifo_counted_resized
    port map (
         wr_clk     		=> B2H_wr_clk   				,
         wr_en      		=> B2H_wr_en    				,
         din        		=> resized_B2H_wr_din		,
         prog_full  		=> B2H_wr_pfull 				,
         full       		=> B2H_wr_full  				,
         rd_clk     		=> B2H_rd_clk   				,
         rd_en      		=> B2H_rd_en    				,
         dout       		=> resized_B2H_rd_dout     ,
         prog_empty 		=> B2H_rd_pempty   			,
         empty      		=> B2H_rd_empty    			,
         rd_data_count  => B2H_rd_data_count_wire  ,
         wr_data_count  => B2H_wr_data_count_wire  ,
			valid          => B2H_rd_valid				,
         rst        		=> rst      
         );

end architecture Behavioral;
