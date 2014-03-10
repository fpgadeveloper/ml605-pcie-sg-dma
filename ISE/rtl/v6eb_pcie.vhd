----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    09:12:51 01 Feb 2010
-- Design Name: 
-- Module Name:    v6pcieDMA - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- 
-- Revision 1.00 - File Released
-- 
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
library UNISIM;
use UNISIM.VComponents.all;

entity v6pcieDMA is
    generic (
          constant pcieLanes            : integer     := C_NUM_PCIE_LANES
          );
    Port (

			 userclk_66MHz		   			 : IN std_logic;    --66  MHz USER Socket SingleEnded
			 userclk_200MHz_n              : IN std_logic;    --200 MHz USER Socket LVDS N
			 userclk_200MHz_p              : IN std_logic;    --200 MHz USER Socket LVDS P
			
          -- DPR blinker
          LEDs_IO_pin                   : OUT   std_logic_vector(7 downto 0);


          -- PCIe transceivers
          pci_exp_rxp                   : IN    std_logic_vector(pcieLanes - 1 downto 0);
          pci_exp_rxn                   : IN    std_logic_vector(pcieLanes - 1 downto 0);
          pci_exp_txp                   : OUT   std_logic_vector(pcieLanes - 1 downto 0);
          pci_exp_txn                   : OUT   std_logic_vector(pcieLanes - 1 downto 0);

          -- Necessity signals
          sys_clk_p                     : IN    std_logic; --125 MHz PCIe Clock
          sys_clk_n                     : IN    std_logic; --125 MHz PCIe Clock
          sys_reset_n                   : IN    std_logic  --Reset
          );                       

end entity v6pcieDMA;


architecture Behavioral of v6pcieDMA is


 component PCIe_UserLogic_00
  port (
    bram_rd_dout: in std_logic_vector(63 downto 0); 
    debug_in_1i: in std_logic_vector(31 downto 0); 
    debug_in_2i: in std_logic_vector(31 downto 0); 
    debug_in_3i: in std_logic_vector(31 downto 0); 
    debug_in_4i: in std_logic_vector(31 downto 0); 
    dma_host2board_busy: in std_logic; 
    dma_host2board_done: in std_logic; 
    fifo_rd_count: in std_logic_vector(14 downto 0); 
    fifo_wr_count: in std_logic_vector(14 downto 0); 
    fifo_rd_dout: in std_logic_vector(71 downto 0); 
    fifo_rd_empty: in std_logic; 
    fifo_rd_pempty: in std_logic; 
    fifo_wr_full: in std_logic; 
    fifo_wr_pfull: in std_logic; 
    fifo_rd_valid: in std_logic; 
    inout_logic_cw_ce: in std_logic := '1'; 
    inout_logic_cw_clk: in std_logic; 
    reg01_td: in std_logic_vector(31 downto 0); 
    reg01_tv: in std_logic; 
    reg02_td: in std_logic_vector(31 downto 0); 
    reg02_tv: in std_logic; 
    reg03_td: in std_logic_vector(31 downto 0); 
    reg03_tv: in std_logic; 
    reg04_td: in std_logic_vector(31 downto 0); 
    reg04_tv: in std_logic; 
    reg05_td: in std_logic_vector(31 downto 0); 
    reg05_tv: in std_logic; 
    reg06_td: in std_logic_vector(31 downto 0); 
    reg06_tv: in std_logic; 
    reg07_td: in std_logic_vector(31 downto 0); 
    reg07_tv: in std_logic; 
    reg08_td: in std_logic_vector(31 downto 0); 
    reg08_tv: in std_logic; 
    reg09_td: in std_logic_vector(31 downto 0); 
    reg09_tv: in std_logic; 
    reg10_td: in std_logic_vector(31 downto 0); 
    reg10_tv: in std_logic; 
    reg11_td: in std_logic_vector(31 downto 0); 
    reg11_tv: in std_logic; 
    reg12_td: in std_logic_vector(31 downto 0); 
    reg12_tv: in std_logic; 
    reg13_td: in std_logic_vector(31 downto 0); 
    reg13_tv: in std_logic; 
    reg14_td: in std_logic_vector(31 downto 0); 
    reg14_tv: in std_logic; 
    rst_i: in std_logic; 
    user_logic_cw_ce: in std_logic := '1'; 
    user_logic_cw_clk: in std_logic; 
    bram_rd_addr: out std_logic_vector(11 downto 0); 
    bram_wr_addr: out std_logic_vector(11 downto 0); 
    bram_wr_din: out std_logic_vector(63 downto 0); 
    bram_wr_en: out std_logic_vector(7 downto 0); 
    fifo_rd_en: out std_logic; 
    fifo_wr_din: out std_logic_vector(71 downto 0); 
    fifo_wr_en: out std_logic; 
    reg01_rd: out std_logic_vector(31 downto 0); 
    reg01_rv: out std_logic; 
    reg02_rd: out std_logic_vector(31 downto 0); 
    reg02_rv: out std_logic; 
    reg03_rd: out std_logic_vector(31 downto 0); 
    reg03_rv: out std_logic; 
    reg04_rd: out std_logic_vector(31 downto 0); 
    reg04_rv: out std_logic; 
    reg05_rd: out std_logic_vector(31 downto 0); 
    reg05_rv: out std_logic; 
    reg06_rd: out std_logic_vector(31 downto 0); 
    reg06_rv: out std_logic; 
    reg07_rd: out std_logic_vector(31 downto 0); 
    reg07_rv: out std_logic; 
    reg08_rd: out std_logic_vector(31 downto 0); 
    reg08_rv: out std_logic; 
    reg09_rd: out std_logic_vector(31 downto 0); 
    reg09_rv: out std_logic; 
    reg10_rd: out std_logic_vector(31 downto 0); 
    reg10_rv: out std_logic; 
    reg11_rd: out std_logic_vector(31 downto 0); 
    reg11_rv: out std_logic; 
    reg12_rd: out std_logic_vector(31 downto 0); 
    reg12_rv: out std_logic; 
    reg13_rd: out std_logic_vector(31 downto 0); 
    reg13_rv: out std_logic; 
    reg14_rd: out std_logic_vector(31 downto 0); 
    reg14_rv: out std_logic; 
    rst_o: out std_logic; 
    user_int_1o: out std_logic; 
    user_int_2o: out std_logic;
    user_int_3o: out std_logic
  );
end component;


-- -----------------------------------------------------------------------
--- COMPONENT Declaration: v6_pcie_v1_6 x4 									  ---
--- OSS: Ricordarsi di matchare POWER_SAVE - VENDOR_ID e DEVICE_ID     ---
--- OSS: For POWER_SAVE error correct bit[4] and install ISE12 Patch!! ---
-- -----------------------------------------------------------------------

--S component v6_pcie_v1_7_x1
   component v6_pcie_v1_7_x4
   generic (
         PL_FAST_TRAIN  : boolean
         );
   port (
    ---------------------------------------------------------
    -- 1. PCI Express (pci_exp) Interface
    ---------------------------------------------------------

    -- Tx
    pci_exp_txn                    : out STD_LOGIC_VECTOR ( pcieLanes - 1 downto 0 ); 
    pci_exp_txp                    : out STD_LOGIC_VECTOR ( pcieLanes - 1 downto 0 ); 
                  
    -- Rx
    pci_exp_rxn                    : in  STD_LOGIC_VECTOR ( pcieLanes - 1 downto 0 ); 
    pci_exp_rxp                    : in  STD_LOGIC_VECTOR ( pcieLanes - 1 downto 0 ); 
                  
    ---------------------------------------------------------
    -- 2. Transaction (TRN) Interface
    ---------------------------------------------------------

    -- Common
    trn_clk                        : out STD_LOGIC; 
    trn_reset_n                    : out STD_LOGIC; 
    trn_lnk_up_n                   : out STD_LOGIC; 

    -- Tx                      
    trn_tsof_n                     : in  STD_LOGIC; 
    trn_teof_n                     : in  STD_LOGIC; 
    trn_td                         : in  STD_LOGIC_vector (64-1 downto 0); 
    trn_trem_n                     : in  STD_LOGIC; 
    trn_tsrc_rdy_n                 : in  STD_LOGIC; 
    trn_tsrc_dsc_n                 : in  STD_LOGIC; 
    trn_tbuf_av                    : out STD_LOGIC_vector (6-1 downto 0); 
    trn_terrfwd_n                  : in  STD_LOGIC; 

    trn_tcfg_req_n                 : out STD_LOGIC;
    trn_terr_drop_n                : out STD_LOGIC; 
    trn_tdst_rdy_n                 : out STD_LOGIC; 
    trn_tcfg_gnt_n                 : in  STD_LOGIC; 
    trn_tstr_n                     : in  STD_LOGIC; 

    -- Rx                      
    trn_rd                         : out STD_LOGIC_vector (64-1 downto 0); 
    trn_rrem_n                     : out STD_LOGIC; 
    trn_rsof_n                     : out STD_LOGIC; 
    trn_reof_n                     : out STD_LOGIC; 
    trn_rsrc_rdy_n                 : out STD_LOGIC; 
    trn_rsrc_dsc_n                 : out STD_LOGIC; 
    trn_rerrfwd_n                  : out STD_LOGIC; 
    trn_rbar_hit_n                 : out STD_LOGIC_vector (7-1 downto 0); 
    trn_rdst_rdy_n                 : in  STD_LOGIC; 
    trn_rnp_ok_n                   : in  STD_LOGIC; 

    -- Flow Control            
    trn_fc_cpld                    : out STD_LOGIC_vector (12-1 downto 0); 
    trn_fc_cplh                    : out STD_LOGIC_vector (8-1 downto 0); 
    trn_fc_npd                     : out STD_LOGIC_vector (12-1 downto 0); 
    trn_fc_nph                     : out STD_LOGIC_vector (8-1 downto 0); 
    trn_fc_pd                      : out STD_LOGIC_vector (12-1 downto 0); 
    trn_fc_ph                      : out STD_LOGIC_vector (8-1 downto 0); 
    trn_fc_sel                     : in  STD_LOGIC_vector (3-1 downto 0); 
                               

    ---------------------------------------------------------
    -- 3. Configuration (CFG) Interface
    ---------------------------------------------------------

    cfg_do                         : out STD_LOGIC_vector (32-1 downto 0); 
    cfg_rd_wr_done_n               : out STD_LOGIC; 
    cfg_di                         : in  STD_LOGIC_vector (32-1 downto 0); 
    cfg_byte_en_n                  : in  STD_LOGIC_vector (4-1 downto 0); 
    cfg_dwaddr                     : in  STD_LOGIC_vector (10-1 downto 0); 
    cfg_wr_en_n                    : in  STD_LOGIC; 
    cfg_rd_en_n                    : in  STD_LOGIC; 

    cfg_err_cor_n                  : in  STD_LOGIC; 
    cfg_err_ur_n                   : in  STD_LOGIC; 
    cfg_err_ecrc_n                 : in  STD_LOGIC; 
    cfg_err_cpl_timeout_n          : in  STD_LOGIC; 
    cfg_err_cpl_abort_n            : in  STD_LOGIC; 
    cfg_err_cpl_unexpect_n         : in  STD_LOGIC; 
    cfg_err_posted_n               : in  STD_LOGIC; 
    cfg_err_locked_n               : in  STD_LOGIC; 
    cfg_err_tlp_cpl_header         : in  STD_LOGIC_vector (48-1 downto 0); 
    cfg_err_cpl_rdy_n              : out STD_LOGIC; 
    cfg_interrupt_n                : in  STD_LOGIC; 
    cfg_interrupt_rdy_n            : out STD_LOGIC; 
    cfg_interrupt_assert_n         : in  STD_LOGIC; 
    cfg_interrupt_di               : in  STD_LOGIC_vector (8-1 downto 0); 
    cfg_interrupt_do               : out STD_LOGIC_vector (8-1 downto 0); 
    cfg_interrupt_mmenable         : out STD_LOGIC_vector (3-1 downto 0); 
    cfg_interrupt_msienable        : out STD_LOGIC; 
    cfg_interrupt_msixenable       : out STD_LOGIC; 
    cfg_interrupt_msixfm           : out STD_LOGIC; 
    cfg_turnoff_ok_n               : in  STD_LOGIC; 
    cfg_to_turnoff_n               : out STD_LOGIC; 
    cfg_trn_pending_n              : in  STD_LOGIC; 
    cfg_pm_wake_n                  : in  STD_LOGIC; 
    cfg_bus_number                 : out STD_LOGIC_vector (8-1 downto 0); 
    cfg_device_number              : out STD_LOGIC_vector (5-1 downto 0); 
    cfg_function_number            : out STD_LOGIC_vector (3-1 downto 0); 
    cfg_status                     : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_command                    : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_dstatus                    : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_dcommand                   : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_lstatus                    : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_lcommand                   : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_dcommand2                  : out STD_LOGIC_vector (16-1 downto 0); 
    cfg_pcie_link_state_n          : out STD_LOGIC_vector (3-1 downto 0); 
    cfg_dsn                        : in  STD_LOGIC_vector (64-1 downto 0); 
    cfg_pmcsr_pme_en               : out STD_LOGIC; 
    cfg_pmcsr_pme_status           : out STD_LOGIC; 
    cfg_pmcsr_powerstate           : out STD_LOGIC_vector (2-1 downto 0); 
--S v1.3 --> 1.6    lnk_clk_en     : out STD_LOGIC; 
--	 lnk_clk_en     					  : out STD_LOGIC; 

    ---------------------------------------------------------
    -- 4. Physical Layer Control and Status (PL) Interface
    ---------------------------------------------------------

    pl_initial_link_width          : out STD_LOGIC_vector (3-1 downto 0);
    pl_lane_reversal_mode          : out STD_LOGIC_vector (2-1 downto 0);
    pl_link_gen2_capable           : out STD_LOGIC;
    pl_link_partner_gen2_supported : out STD_LOGIC;
    pl_link_upcfg_capable          : out STD_LOGIC;
    pl_ltssm_state                 : out STD_LOGIC_vector (6-1 downto 0);
    pl_received_hot_rst            : out STD_LOGIC;
    pl_sel_link_rate               : out STD_LOGIC;
    pl_sel_link_width              : out STD_LOGIC_vector (2-1 downto 0);
    pl_directed_link_auton         : in  STD_LOGIC;
    pl_directed_link_change        : in  STD_LOGIC_vector (2-1 downto 0);
    pl_directed_link_speed         : in  STD_LOGIC;
    pl_directed_link_width         : in  STD_LOGIC_vector (2-1 downto 0);
    pl_upstream_prefer_deemph      : in  STD_LOGIC;


    ---------------------------------------------------------
    -- 5. System  (SYS) Interface
    ---------------------------------------------------------

    sys_clk                        : in  STD_LOGIC;
    sys_reset_n                    : in  STD_LOGIC
  );
 end component;





     signal fifo_reset_done              : std_logic;
     signal pio_reading_status           : std_logic;


-- -----------------------------------------------------------------------
--  DDR SDRAM control module
-- -----------------------------------------------------------------------

   COMPONENT bram_DDRs_Control_loopback
   GENERIC (
             C_ASYNFIFO_WIDTH  :  integer ;
             P_SIMULATION      :  boolean
            );
   PORT (

      DDR_wr_sof               : IN    std_logic;
      DDR_wr_eof               : IN    std_logic;
      DDR_wr_v                 : IN    std_logic;
      DDR_wr_FA                : IN    std_logic;
      DDR_wr_Shift             : IN    std_logic;
      DDR_wr_Mask              : IN    std_logic_vector(2-1 downto 0);
      DDR_wr_din               : IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_wr_full              : OUT   std_logic;

      DDR_rdc_sof              : IN    std_logic;
      DDR_rdc_eof              : IN    std_logic;
      DDR_rdc_v                : IN    std_logic;
      DDR_rdc_FA               : IN    std_logic;
      DDR_rdc_Shift            : IN    std_logic;
      DDR_rdc_din              : IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_rdc_full             : OUT   std_logic;

      -- DDR payload FIFO Read Port
      DDR_FIFO_RdEn            : IN    std_logic;
      DDR_FIFO_Empty           : OUT   std_logic;
      DDR_FIFO_RdQout          : OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);

      -- Common interface
      DDR_Ready                : OUT   std_logic;
      DDR_Blinker              : OUT   std_logic;
      mem_clk                  : IN    std_logic;
      trn_clk                  : IN    std_logic;
		Sim_Zeichen              : OUT   std_logic;  
      trn_reset_n              : IN    std_logic
    );
   END COMPONENT;	
	
	
   COMPONENT bram_DDRs_Control
   GENERIC (
             C_ASYNFIFO_WIDTH  :  integer ;
             P_SIMULATION      :  boolean
            );
   PORT (

		--USER Logic Interface
		user_wr_weA              : IN    std_logic_vector(7 downto 0);
		user_wr_addrA            : IN    std_logic_vector(C_PRAM_AWIDTH-1 downto 0);
		user_wr_dinA             : IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
		user_rd_addrB            : IN    std_logic_vector(C_PRAM_AWIDTH-1 downto 0);
		user_rd_doutB            : OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);
		user_rd_clk              : IN    std_logic;
		user_wr_clk              : IN    std_logic;

      -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
      DDR_wr_sof               : IN    std_logic;
      DDR_wr_eof               : IN    std_logic;
      DDR_wr_v                 : IN    std_logic;
      DDR_wr_FA                : IN    std_logic;
      DDR_wr_Shift             : IN    std_logic;
      DDR_wr_Mask              : IN    std_logic_vector(2-1 downto 0);
      DDR_wr_din               : IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_wr_full              : OUT   std_logic;

      DDR_rdc_sof              : IN    std_logic;
      DDR_rdc_eof              : IN    std_logic;
      DDR_rdc_v                : IN    std_logic;
      DDR_rdc_FA               : IN    std_logic;
      DDR_rdc_Shift            : IN    std_logic;
      DDR_rdc_din              : IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_rdc_full             : OUT   std_logic;

      -- DDR payload FIFO Read Port
      DDR_FIFO_RdEn            : IN    std_logic;
      DDR_FIFO_Empty           : OUT   std_logic;
      DDR_FIFO_RdQout          : OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);

      -- Common interface
      DDR_Ready                : OUT   std_logic;
      DDR_Blinker              : OUT   std_logic;
      mem_clk                  : IN    std_logic;
      trn_clk                  : IN    std_logic;
		Sim_Zeichen              : OUT   std_logic;  
      trn_reset_n              : IN    std_logic
    );
   END COMPONENT;
	
	
   signal    DDR_wr_sof               :  std_logic;
   signal    DDR_wr_eof               :  std_logic;
   signal    DDR_wr_v                 :  std_logic;
   signal    DDR_wr_FA                :  std_logic;
   signal    DDR_wr_Shift             :  std_logic;
   signal    DDR_wr_Mask              :  std_logic_vector(2-1 downto 0);
   signal    DDR_wr_din               :  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
   signal    DDR_wr_full              :  std_logic;

   signal    DDR_rdc_sof              :  std_logic;
   signal    DDR_rdc_eof              :  std_logic;
   signal    DDR_rdc_v                :  std_logic;
   signal    DDR_rdc_FA               :  std_logic;
   signal    DDR_rdc_Shift            :  std_logic;
   signal    DDR_rdc_din              :  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
   signal    DDR_rdc_full             :  std_logic;

   signal    DDR_FIFO_RdEn            :  std_logic; 
   signal    DDR_FIFO_Empty           :  std_logic;
   signal    DDR_FIFO_RdQout          :  std_logic_vector(C_DBUS_WIDTH-1 downto 0);

   signal    DDR_Ready                :  std_logic;
   signal    DDR_Blinker              :  std_logic;

	signal	user_wr_weA              : std_logic_vector(7 downto 0) := (Others =>'0');
	signal	user_wr_addrA            : std_logic_vector(C_PRAM_AWIDTH-1 downto 0) := (Others =>'0');
	signal	user_wr_dinA             : std_logic_vector(C_DBUS_WIDTH-1 downto 0)  := (Others =>'0');
	signal	user_rd_addrB            : std_logic_vector(C_PRAM_AWIDTH-1 downto 0) := (Others =>'0');
	signal	user_rd_doutB            : std_logic_vector(C_DBUS_WIDTH-1 downto 0);


   -- -----------------------------------------------------------------------
   -- FIFO module
   -- -----------------------------------------------------------------------


   component eb_wrapper_loopback
     port (
           wr_clk      : IN  std_logic;
           wr_en       : IN  std_logic;
           din         : IN  std_logic_VECTOR(72-1 downto 0);
           pfull       : OUT std_logic;
           full        : OUT std_logic;

           rd_clk      : IN  std_logic;
           rd_en       : IN  std_logic;
           dout        : OUT std_logic_VECTOR(72-1 downto 0);
           pempty      : OUT std_logic;
           empty       : OUT std_logic;

           data_count  : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
           rst         : IN  std_logic
           );
   end component;


   component eb_wrapper
     port (
			 --FIFO PCIe-->USER
			 H2B_wr_clk        : IN  std_logic;
          H2B_wr_en         : IN  std_logic;
          H2B_wr_din        : IN  std_logic_VECTOR(72-1 downto 0);
          H2B_wr_pfull      : OUT std_logic;
          H2B_wr_full       : OUT std_logic;
          H2B_wr_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
          H2B_rd_clk        : IN  std_logic;
          H2B_rd_en         : IN  std_logic;
          H2B_rd_dout       : OUT std_logic_VECTOR(72-1 downto 0);
          H2B_rd_pempty     : OUT std_logic;
          H2B_rd_empty      : OUT std_logic;
          H2B_rd_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
			 H2B_rd_valid      : OUT std_logic;
			 --FIFO USER-->PCIe
          B2H_wr_clk        : IN  std_logic;
          B2H_wr_en         : IN  std_logic;
          B2H_wr_din        : IN  std_logic_VECTOR(72-1 downto 0);
          B2H_wr_pfull      : OUT std_logic;
          B2H_wr_full       : OUT std_logic;
          B2H_wr_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0); 
          B2H_rd_clk        : IN  std_logic;
          B2H_rd_en         : IN  std_logic;
          B2H_rd_dout       : OUT std_logic_VECTOR(72-1 downto 0);
          B2H_rd_pempty     : OUT std_logic;
          B2H_rd_empty      : OUT std_logic;
          B2H_rd_data_count : OUT std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
			 B2H_rd_valid		 : OUT std_logic;	
          --RESET from PCIe
			 rst               : IN  std_logic
          );
   end component;


   signal  eb_wclk            :  std_logic;
   signal  eb_we              :  std_logic;
   signal  eb_wsof            :  std_logic;
   signal  eb_weof            :  std_logic;
   signal  eb_din             :  std_logic_VECTOR(72-1 downto 0);
   signal  eb_pfull           :  std_logic;
   signal  eb_full            :  std_logic;
   signal  eb_rclk            :  std_logic;
   signal  eb_re              :  std_logic;
   signal  eb_dout            :  std_logic_VECTOR(72-1 downto 0);
   signal  eb_pempty          :  std_logic;
   signal  eb_empty           :  std_logic;
   signal  eb_valid           :  std_logic;
   signal  eb_rst             :  std_logic;

   signal  eb_data_count      :  std_logic_vector(C_FIFO_DC_WIDTH downto 0);
   signal  H2B_wr_data_count  :  std_logic_vector(C_FIFO_DC_WIDTH downto 0);
   signal  B2H_rd_data_count  :  std_logic_vector(C_FIFO_DC_WIDTH downto 0);


   signal  pio_read_status    :  std_logic;
   signal  eb_FIFO_ow         :  std_logic;

   signal  eb_FIFO_Status     :  std_logic_VECTOR(C_DBUS_WIDTH-1 downto 0);
   signal  H2B_FIFO_Status    :  std_logic_VECTOR(C_DBUS_WIDTH-1 downto 0);
   signal  B2H_FIFO_Status    :  std_logic_VECTOR(C_DBUS_WIDTH-1 downto 0);

   signal  eb_we_up           :  std_logic;
   signal  eb_din_up          :  std_logic_VECTOR(72-1 downto 0);

   signal  tab_sel            : STD_LOGIC;
	
	signal   user_rd_en         : std_logic := '0';
	signal   user_rd_dout       : std_logic_VECTOR(72-1 downto 0);
	signal   user_rd_pempty     : std_logic;
	signal   user_rd_empty      : std_logic;
	signal   user_rd_data_count : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
	signal   user_wr_data_count : std_logic_VECTOR(C_EMU_FIFO_DC_WIDTH-1 downto 0);
	signal   user_wr_en         : std_logic := '0';
	signal   user_wr_din        : std_logic_VECTOR(72-1 downto 0) := (Others =>'0');
	signal   user_wr_pfull      : std_logic;
	signal   user_wr_full       : std_logic;
	signal   user_rd_valid      : std_logic;



------------- COMPONENT Declaration: tlpControl   ------
-- 
 component tlpControl 
   port (
        --  Test pin, emulating DDR data flow discontinuity
        mbuf_UserFull                : IN  std_logic;
        trn_Blinker                  : OUT std_logic;



--S	SIMONE: Wanxau UserLogic Signals, not Used
        -- DCB protocol interface
        protocol_link_act            : IN  std_logic_vector(2-1 downto 0);
        protocol_rst                 : OUT std_logic;
        -- Fabric side: CTL Rx
        ctl_rv                       : OUT std_logic;
        ctl_rd                       : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        -- Fabric side: CTL Tx
        ctl_ttake                    : OUT std_logic;
        ctl_tv                       : IN  std_logic;
        ctl_td                       : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        ctl_tstop                    : OUT std_logic;
        ctl_reset                    : OUT std_logic;
        ctl_status                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        -- Fabric side: DLM Rx
        dlm_rv                       : OUT std_logic;
        dlm_rd                       : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        -- Fabric side: DLM Tx
        dlm_tv                       : IN  std_logic;
        dlm_td                       : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        Link_Buf_full                : IN  std_logic;
        -- Data generator table write
        tab_we                       : OUT std_logic_vector(2-1 downto 0);
        tab_wa                       : OUT std_logic_vector(12-1 downto 0);
        tab_wd                       : OUT std_logic_vector(C_DBUS_WIDTH-1 downto 0);
        -- Data generator control
        DG_is_Running                : IN  std_logic;
        DG_Reset                     : OUT std_logic;
        DG_Mask                      : OUT std_logic;
--S	SIMONE: Wanxau UserLogic Signals, not Used


        -- Interrupter triggers
        DAQ_irq                      : IN  std_logic;
        CTL_irq                      : IN  std_logic;
        DLM_irq                      : IN  std_logic;


        -- SIMONE Register: PC-->FPGA
        reg01_tv                   : OUT std_logic;
        reg01_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg02_tv                   : OUT std_logic;
        reg02_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg03_tv                   : OUT std_logic;
        reg03_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg04_tv                   : OUT std_logic;
        reg04_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg05_tv                   : OUT std_logic;
        reg05_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg06_tv                   : OUT std_logic;
        reg06_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg07_tv                   : OUT std_logic;
        reg07_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg08_tv                   : OUT std_logic;
        reg08_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg09_tv                   : OUT std_logic;
        reg09_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg10_tv                   : OUT std_logic;
        reg10_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg11_tv                   : OUT std_logic;
        reg11_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg12_tv                   : OUT std_logic;
        reg12_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg13_tv                   : OUT std_logic;
        reg13_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg14_tv                   : OUT std_logic;
        reg14_td                   : OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);

        -- SIMONE Register: FPGA-->PC
        reg01_rv                   : IN  std_logic;
        reg01_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg02_rv                   : IN  std_logic;
        reg02_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg03_rv                   : IN  std_logic;
        reg03_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg04_rv                   : IN  std_logic;
        reg04_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg05_rv                   : IN  std_logic;
        reg05_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg06_rv                   : IN  std_logic;
        reg06_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg07_rv                   : IN  std_logic;
        reg07_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg08_rv                   : IN  std_logic;
        reg08_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg09_rv                   : IN  std_logic;
        reg09_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg10_rv                   : IN  std_logic;
        reg10_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg11_rv                   : IN  std_logic;
        reg11_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg12_rv                   : IN  std_logic;
        reg12_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg13_rv                   : IN  std_logic;
        reg13_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
        reg14_rv                   : IN  std_logic;
        reg14_rd                   : IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);

		  --SIMONE debug signals
		 debug_in_1i					  : OUT std_logic_vector(31 downto 0); 
		 debug_in_2i					  : OUT std_logic_vector(31 downto 0); 
		 debug_in_3i					  : OUT std_logic_vector(31 downto 0); 		  
		 debug_in_4i					  : OUT std_logic_vector(31 downto 0); 		  
			

        -- Event Buffer FIFO interface
        eb_FIFO_we                   : OUT std_logic; 
        eb_FIFO_wsof                 : OUT std_logic; 
        eb_FIFO_weof                 : OUT std_logic; 
        eb_FIFO_din                  : OUT std_logic_vector(C_DBUS_WIDTH-1 downto 0);

        eb_FIFO_re                   : OUT std_logic; 
        eb_FIFO_empty                : IN  std_logic; 
        eb_FIFO_qout                 : IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
        eb_FIFO_data_count           : IN  std_logic_vector(C_FIFO_DC_WIDTH downto 0);

        eb_FIFO_ow                   : IN  std_logic;

        pio_reading_status           : OUT std_logic; 
        eb_FIFO_Status               : IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
        eb_FIFO_Rst                  : OUT std_logic;

		  H2B_FIFO_Status    			 : IN std_logic_VECTOR(C_DBUS_WIDTH-1 downto 0);
		  B2H_FIFO_Status    			 : IN std_logic_VECTOR(C_DBUS_WIDTH-1 downto 0);

        -- Debugging signals
        DMA_us_Done                  : OUT std_logic;
        DMA_us_Busy                  : OUT std_logic;
        DMA_us_Busy_LED              : OUT std_logic;
        DMA_ds_Done                  : OUT std_logic;
        DMA_ds_Busy                  : OUT std_logic;
        DMA_ds_Busy_LED              : OUT std_logic;

        -- DDR control interface
        DDR_Ready                    : IN    std_logic;

        DDR_wr_sof                   : OUT   std_logic;
        DDR_wr_eof                   : OUT   std_logic;
        DDR_wr_v                     : OUT   std_logic;
        DDR_wr_FA                    : OUT   std_logic;
        DDR_wr_Shift                 : OUT   std_logic;
        DDR_wr_Mask                  : OUT   std_logic_vector(2-1 downto 0);
        DDR_wr_din                   : OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);
        DDR_wr_full                  : IN    std_logic;

        DDR_rdc_sof                  : OUT   std_logic;
        DDR_rdc_eof                  : OUT   std_logic;
        DDR_rdc_v                    : OUT   std_logic;
        DDR_rdc_FA                   : OUT   std_logic;
        DDR_rdc_Shift                : OUT   std_logic;
        DDR_rdc_din                  : OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);
        DDR_rdc_full                 : IN    std_logic;

        -- DDR payload FIFO Read Port
        DDR_FIFO_RdEn                : OUT std_logic; 
        DDR_FIFO_Empty               : IN  std_logic;
        DDR_FIFO_RdQout              : IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);

        -- Transaction layer interface
        trn_lnk_up_n                 : IN  std_logic;
        trn_rsrc_dsc_n               : IN  std_logic;
        trn_rnp_ok_n                 : OUT std_logic;
        trn_tsrc_dsc_n               : OUT std_logic;
        trn_tdst_dsc_n               : IN  std_logic;
        trn_tbuf_av                  : IN  std_logic_vector(C_TBUF_AWIDTH-1 downto 0);
        trn_terrfwd_n                : OUT std_logic;

        trn_clk                      : IN  std_logic;
        trn_reset_n                  : IN  std_logic;
        trn_rsrc_rdy_n               : IN  std_logic;
        trn_tdst_rdy_n               : IN  std_logic;
        trn_rsof_n                   : IN  std_logic;
        trn_reof_n                   : IN  std_logic;
        trn_rerrfwd_n                : IN  std_logic;
        trn_rrem_n                   : IN  std_logic_vector(C_DBUS_WIDTH/8-1 downto 0);
        trn_rd                       : IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);

        cfg_dcommand                 : IN  std_logic_vector(15 downto 0);
        pcie_link_width              : IN  std_logic_vector( 5 downto 0);
        localId                      : IN  std_logic_vector(15 downto 0);

        cfg_interrupt_n              : OUT std_logic;
        cfg_interrupt_rdy_n          : IN  std_logic;
        cfg_interrupt_mmenable       : IN  std_logic_vector(2 downto 0);
        cfg_interrupt_msienable      : IN  std_logic;
        cfg_interrupt_di             : OUT std_logic_vector(7 downto 0);
        cfg_interrupt_do             : IN  std_logic_vector(7 downto 0);
        cfg_interrupt_assert_n       : OUT std_logic;

        Format_Shower                : OUT   std_logic;

        trn_rbar_hit_n               : IN  std_logic_vector(6 downto 0);
        trn_tsrc_rdy_n               : OUT std_logic;
        trn_rdst_rdy_n               : OUT std_logic;
        trn_tsof_n                   : OUT std_logic;
        trn_teof_n                   : OUT std_logic;
        trn_trem_n                   : OUT std_logic_vector(C_DBUS_WIDTH/8-1 downto 0);
        trn_td                       : OUT std_logic_vector(C_DBUS_WIDTH-1 downto 0)
        );
 end component;

 signal   Format_Shower              : std_logic;




  -- TRN Layer signals

  signal trn_terr_drop_n                 : std_logic;
  signal trn_tcfg_gnt_n                  : std_logic;
  signal trn_tstr_n                      : std_logic;
  signal trn_fc_cpld                     : STD_LOGIC_vector (12-1 downto 0); 
  signal trn_fc_cplh                     : STD_LOGIC_vector (8-1 downto 0); 
  signal trn_fc_npd                      : STD_LOGIC_vector (12-1 downto 0); 
  signal trn_fc_nph                      : STD_LOGIC_vector (8-1 downto 0); 
  signal trn_fc_pd                       : STD_LOGIC_vector (12-1 downto 0); 
  signal trn_fc_ph                       : STD_LOGIC_vector (8-1 downto 0); 
  signal trn_fc_sel                      : STD_LOGIC_vector (3-1 downto 0); 
                                    
  signal cfg_interrupt_msixenable        : std_logic;
  signal cfg_interrupt_msixfm            : std_logic;
  signal cfg_dcommand2                   : std_logic_vector (16-1 downto 0);
  signal trn_tcfg_req_n                  : std_logic;
 
  signal  pl_initial_link_width          : STD_LOGIC_vector (3-1 downto 0);
  signal  pl_lane_reversal_mode          : STD_LOGIC_vector (2-1 downto 0);
  signal  pl_link_gen2_capable           : STD_LOGIC;
  signal  pl_link_partner_gen2_supported : STD_LOGIC;
  signal  pl_link_upcfg_capable          : STD_LOGIC;
  signal  pl_ltssm_state                 : STD_LOGIC_vector (6-1 downto 0);
  signal  pl_received_hot_rst            : STD_LOGIC;
  signal  pl_sel_link_rate               : STD_LOGIC;
  signal  pl_sel_link_width              : STD_LOGIC_vector (2-1 downto 0);
  signal  pl_directed_link_auton         : STD_LOGIC;
  signal  pl_directed_link_change        : STD_LOGIC_vector (2-1 downto 0);
  signal  pl_directed_link_speed         : STD_LOGIC;
  signal  pl_directed_link_width         : STD_LOGIC_vector (2-1 downto 0);
  signal  pl_upstream_prefer_deemph      : STD_LOGIC;

  signal  trn_reset_n_int1       : STD_LOGIC;
  signal  trn_lnk_up_n_int1      : STD_LOGIC;

  signal trn_clk                     : std_logic;
  signal trn_reset_n                 : std_logic;
  signal trn_lnk_up_n                : std_logic;
  signal trn_td                      : std_logic_vector(63 downto 0);
  signal trn_trem_n                  : std_logic_vector(7 downto 0);
  signal trn_tsof_n                  : std_logic;
  signal trn_teof_n                  : std_logic;
  signal trn_tsrc_rdy_n              : std_logic;
  signal trn_tdst_rdy_n              : std_logic;
  signal trn_tdst_dsc_n              : std_logic;
  signal trn_tsrc_dsc_n              : std_logic;
  signal trn_terrfwd_n               : std_logic;
  signal trn_tbuf_av                 : std_logic_vector(5 downto 0);
  signal trn_rd                      : std_logic_vector(63 downto 0);
  signal trn_rrem_n                  : std_logic_vector(7 downto 0);
  signal trn_rsof_n                  : std_logic;
  signal trn_reof_n                  : std_logic;
  signal trn_rsrc_rdy_n              : std_logic;
  signal trn_rsrc_dsc_n              : std_logic;
  signal trn_rdst_rdy_n              : std_logic;
  signal trn_rerrfwd_n               : std_logic;
  signal trn_rnp_ok_n                : std_logic;
  signal trn_rbar_hit_n              : std_logic_vector(6 downto 0);
  signal trn_rfc_nph_av              : std_logic_vector(7 downto 0);
  signal trn_rfc_npd_av              : std_logic_vector(11 downto 0);
  signal trn_rfc_ph_av               : std_logic_vector(7 downto 0);
  signal trn_rfc_pd_av               : std_logic_vector(11 downto 0);
  signal trn_rfc_cplh_av             : std_logic_vector(7 downto 0);
  signal trn_rfc_cpld_av             : std_logic_vector(11 downto 0);
  signal trn_rcpl_streaming_n        : std_logic;
  signal cfg_do                      : std_logic_vector(31 downto 0);
  signal cfg_rd_wr_done_n            : std_logic;
  signal cfg_di                      : std_logic_vector(31 downto 0);
  signal cfg_byte_en_n               : std_logic_vector(3 downto 0);
  signal cfg_dwaddr                  : std_logic_vector(9 downto 0);
  signal cfg_wr_en_n                 : std_logic;
  signal cfg_rd_en_n                 : std_logic;
  signal cfg_err_cor_n               : std_logic;
  signal cfg_err_ur_n                : std_logic;
  signal cfg_err_cpl_rdy_n           : std_logic;
  signal cfg_err_ecrc_n              : std_logic;
  signal cfg_err_cpl_timeout_n       : std_logic;
  signal cfg_err_cpl_abort_n         : std_logic;
  signal cfg_err_cpl_unexpect_n      : std_logic;
  signal cfg_err_posted_n            : std_logic;
  signal cfg_err_locked_n            : std_logic;
  signal cfg_err_tlp_cpl_header      : std_logic_vector(47 downto 0);
  signal cfg_interrupt_n             : std_logic;
  signal cfg_interrupt_rdy_n         : std_logic;
  signal cfg_interrupt_mmenable      : std_logic_vector(2 downto 0);
  signal cfg_interrupt_msienable     : std_logic;
  signal cfg_interrupt_di            : std_logic_vector(7 downto 0);
  signal cfg_interrupt_do            : std_logic_vector(7 downto 0);
  signal cfg_interrupt_assert_n      : std_logic;
  signal cfg_turnoff_ok_n            : std_logic;
  signal cfg_to_turnoff_n            : std_logic;
  signal cfg_pm_wake_n               : std_logic;
  signal cfg_pcie_link_state_n       : std_logic_vector(2 downto 0);
  signal cfg_trn_pending_n           : std_logic;
  signal cfg_bus_number              : std_logic_vector(7 downto 0);
  signal cfg_device_number           : std_logic_vector(4 downto 0);
  signal cfg_function_number         : std_logic_vector(2 downto 0);
  signal cfg_dsn                     : std_logic_vector(63 downto 0);
  signal cfg_status                  : std_logic_vector(15 downto 0);
  signal cfg_command                 : std_logic_vector(15 downto 0);
  signal cfg_dstatus                 : std_logic_vector(15 downto 0);
  signal cfg_dcommand                : std_logic_vector(15 downto 0);
  signal cfg_lstatus                 : std_logic_vector(15 downto 0);
  signal cfg_lcommand                : std_logic_vector(15 downto 0);
  signal fast_train_simulation_only  : std_logic;
  signal two_plm_auto_config         : std_logic_vector(1 downto 0);
  signal sys_clk_c                   : std_logic;
  signal sys_reset_n_c               : std_logic;
  signal reset_n                     : std_logic;

  signal localId                     : std_logic_vector(15 downto 0);
  signal pcie_link_width             : std_logic_vector( 5 downto 0);

  signal synclk2out                  : std_logic;

  signal Sim_Zeichen                 : std_logic;
  --
  signal   trn_Blinker               : std_logic;



	signal   DAQ_irq            : std_logic := '0';
	signal   CTL_irq            : std_logic := '0';
	signal   DLM_irq            : std_logic := '0';



--S	SIMONE: Wanxau UserLogic Signals, not Used
	signal   protocol_link_act  : std_logic_vector(2-1 downto 0) := (OTHERS=>'0');
	signal   protocol_rst       : std_logic;
	signal   daq_rstop          : std_logic;
	signal   ctl_rv             : std_logic;
	signal   ctl_rd             : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
	signal   ctl_ttake          : std_logic;
	signal   ctl_tv             : std_logic := '0';
	signal   ctl_td             : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
	signal   ctl_tstop          : std_logic;
	signal   ctl_reset          : std_logic;
	signal   ctl_status         : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
	signal   dlm_tv             : std_logic;
	signal   dlm_td             : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
	signal   dlm_rv             : std_logic := '0';
	signal   dlm_rd             : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal   tab_we             : std_logic_vector(2-1 downto 0);
   signal   tab_wa             : std_logic_vector(12-1 downto 0);
   signal   tab_wd             : std_logic_vector(C_DBUS_WIDTH-1 downto 0);
   signal   dg_running         : std_logic := '0';
   signal   dg_rst             : STD_LOGIC;
   signal   DG_Mask            : STD_LOGIC;
--S	SIMONE: Wanxau UserLogic Signals, not Used



        -- SIMONE Register: PC-->FPGA
   signal  reg01_tv            : std_logic;
   signal  reg01_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg02_tv            : std_logic;
   signal  reg02_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg03_tv            : std_logic;
   signal  reg03_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg04_tv            : std_logic;
   signal  reg04_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg05_tv            : std_logic;
   signal  reg05_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg06_tv            : std_logic;
   signal  reg06_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg07_tv            : std_logic;
   signal  reg07_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg08_tv            : std_logic;
   signal  reg08_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg09_tv            : std_logic;
   signal  reg09_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg10_tv            : std_logic;
   signal  reg10_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg11_tv            : std_logic;
   signal  reg11_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg12_tv            : std_logic;
   signal  reg12_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg13_tv            : std_logic;
   signal  reg13_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
   signal  reg14_tv            : std_logic;
   signal  reg14_td            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);

        -- SIMONE Register: FPGA-->PC
   signal  reg01_rv            : std_logic := '0';
   signal  reg01_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg02_rv            : std_logic := '0';
   signal  reg02_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg03_rv            : std_logic := '0';
   signal  reg03_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg04_rv            : std_logic := '0';
   signal  reg04_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg05_rv            : std_logic := '0';
   signal  reg05_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg06_rv            : std_logic := '0';
   signal  reg06_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg07_rv            : std_logic := '0';
   signal  reg07_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg08_rv            : std_logic := '0';
   signal  reg08_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg09_rv            : std_logic := '0';
   signal  reg09_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg10_rv            : std_logic := '0';
   signal  reg10_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg11_rv            : std_logic := '0';
   signal  reg11_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg12_rv            : std_logic := '0';
   signal  reg12_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg13_rv            : std_logic := '0';
   signal  reg13_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');
   signal  reg14_rv            : std_logic := '0';
   signal  reg14_rd            : std_logic_vector(C_DBUS_WIDTH/2-1 downto 0) := (OTHERS=>'0');

   signal  debug_in_1i			 : std_logic_vector(31 downto 0); 
   signal  debug_in_2i			 : std_logic_vector(31 downto 0); 
   signal  debug_in_3i			 : std_logic_vector(31 downto 0); 
   signal  debug_in_4i			 : std_logic_vector(31 downto 0); 

	signal  user_rst_o          : std_logic;

	signal  clk_200MHz          : std_logic;

	signal  DMA_Host2Board_Busy : std_logic;
	signal  DMA_Host2Board_Done : std_logic;

	signal  DMA_us_Busy         : std_logic;
	signal  DMA_us_Done         : std_logic;
	signal  DMA_ds_Done         : std_logic; 
	signal  DMA_ds_Busy         : std_logic; 
	

begin


   LoopBack_Off_UserLogic:  if not USE_LOOPBACK_TEST generate

--S SIMONE: My Custom User Logic!!
  pcie_userlogic_00_x0: PCIe_UserLogic_00
    port map (
		inout_logic_cw_ce   => '1',
		inout_logic_cw_clk  => trn_clk,
		user_logic_cw_ce    => '1',
		user_logic_cw_clk   => clk_200MHz,
      fifo_rd_count  => user_rd_data_count,
      fifo_rd_dout   => user_rd_dout      ,
      fifo_rd_empty  => user_rd_empty     ,
      fifo_rd_pempty => user_rd_pempty    ,
      fifo_wr_full   => user_wr_full      , 
      fifo_wr_pfull  => user_wr_pfull     ,
      fifo_rd_en 		=> user_rd_en        , 
      fifo_wr_din 	=> user_wr_din       ,
      fifo_wr_en 		=> user_wr_en        ,
      fifo_rd_valid	=> user_rd_valid     ,
      fifo_wr_count  => user_wr_data_count,
      bram_rd_addr 	=> user_rd_addrB     ,
      bram_wr_addr 	=> user_wr_addrA     ,
      bram_wr_din 	=> user_wr_dinA      ,
      bram_wr_en 		=> user_wr_weA       ,
      bram_rd_dout   => user_rd_doutB     , 
		DMA_Host2Board_Busy => DMA_Host2Board_Busy,
		DMA_Host2Board_Done => DMA_Host2Board_Done,
		reg01_td 		=> reg01_td,					        
      reg01_tv 		=> reg01_tv,                     
      reg02_td 		=> reg02_td,                      
      reg02_tv 		=> reg02_tv,                     
      reg03_td 		=> reg03_td,                 
      reg03_tv 		=> reg03_tv,                        
      reg04_td 		=> reg04_td,                 
      reg04_tv 		=> reg04_tv,                        
      reg05_td 		=> reg05_td,                 
      reg05_tv 		=> reg05_tv,                        
      reg06_td 		=> reg06_td,                 
      reg06_tv 		=> reg06_tv,                        
      reg07_td 		=> reg07_td,                 
      reg07_tv 		=> reg07_tv,                        
      reg08_td 		=> reg08_td,                 
      reg08_tv 		=> reg08_tv,                        
      reg09_td 		=> reg09_td,                 
      reg09_tv 		=> reg09_tv,                        
      reg10_td 		=> reg10_td,                 
      reg10_tv 		=> reg10_tv,                        
      reg11_td 		=> reg11_td,                 
      reg11_tv 		=> reg11_tv,                        
      reg12_td 		=> reg12_td,                 
      reg12_tv 		=> reg12_tv,                        
      reg13_td 		=> reg13_td,                 
      reg13_tv 		=> reg13_tv,                        
      reg14_td 		=> reg14_td,                 
      reg14_tv 		=> reg14_tv,                        
      reg01_rd 		=> reg01_rd,                      
      reg01_rv 		=> reg01_rv,                    
      reg02_rd 		=> reg02_rd,                     
      reg02_rv 		=> reg02_rv,
      reg03_rd 		=> reg03_rd,
      reg03_rv 		=> reg03_rv,
      reg04_rd 		=> reg04_rd,
      reg04_rv 		=> reg04_rv,
      reg05_rd 		=> reg05_rd,
      reg05_rv 		=> reg05_rv,
      reg06_rd 		=> reg06_rd,
      reg06_rv 		=> reg06_rv,
      reg07_rd 		=> reg07_rd,
      reg07_rv 		=> reg07_rv,
      reg08_rd 		=> reg08_rd,
      reg08_rv 		=> reg08_rv,
      reg09_rd 		=> reg09_rd,
      reg09_rv 		=> reg09_rv,
      reg10_rd 		=> reg10_rd,
      reg10_rv 		=> reg10_rv,
      reg11_rd 		=> reg11_rd,
      reg11_rv 		=> reg11_rv,
      reg12_rd 		=> reg12_rd,
      reg12_rv 		=> reg12_rv,
      reg13_rd 		=> reg13_rd,
      reg13_rv 		=> reg13_rv,
      reg14_rd 		=> reg14_rd,
      reg14_rv 		=> reg14_rv,
		user_int_1o    => CTL_irq,
		user_int_2o    => DAQ_irq,
		user_int_3o    => DLM_irq,
      debug_in_1i    => debug_in_1i,
      debug_in_2i    => debug_in_2i,
      debug_in_3i    => debug_in_3i,
      debug_in_4i    => debug_in_4i,
      rst_i 			=> trn_reset_n,
      rst_o 			=> user_rst_o
    );

   end generate;

	DMA_Host2Board_Busy <= '0'; --DMA_ds_Busy;
	DMA_Host2Board_Done <= DMA_ds_Done;
   LEDs_IO_pin(5) <= DMA_ds_Done;
	LEDs_IO_pin(7) <= DMA_us_Done;



   sys_reset_n_ibuf : IBUF
      port map (
                 O      =>  sys_reset_n_c,
                 I      =>  sys_reset_n
                );

   refclk_ibuf : IBUFDS_GTXE1
      port map (
                 O      =>  sys_clk_c,
                 ODIV2  =>  open,
                 I      =>  sys_clk_p,
                 IB     =>  sys_clk_n,
                 CEB    =>  '0'
                );

   userclk_ibuf : IBUFDS 
      port map (
                 O  => clk_200MHz,
                 I  => userclk_200MHz_p,
                 IB => userclk_200MHz_n
                );



   cfg_err_cor_n              <= '1';
   cfg_err_ur_n               <= '1';
   cfg_err_ecrc_n             <= '1';
   cfg_err_cpl_timeout_n      <= '1';
   cfg_err_cpl_abort_n        <= '1';
   cfg_err_cpl_unexpect_n     <= '1';
   cfg_err_posted_n           <= '0';
   cfg_err_locked_n           <= '0';
   cfg_err_tlp_cpl_header     <= (OTHERS=>'0');
   cfg_trn_pending_n          <= '1';
   cfg_pm_wake_n              <= '1';
                           

-- 
   trn_fc_sel                 <= (OTHERS=>'0');

   pl_directed_link_auton     <= '0';
   pl_directed_link_change    <= (OTHERS=>'0');
   pl_directed_link_speed     <= '0';
   pl_directed_link_width     <= (OTHERS=>'0');
   pl_upstream_prefer_deemph  <= '0';

   trn_tcfg_gnt_n             <= '0';
   trn_tstr_n                 <= '0';  -- '1';
                       
-- 

   trn_tdst_dsc_n             <= '1';

--
   cfg_di                     <= (OTHERS=>'0');
   cfg_dwaddr                 <= (OTHERS=>'1');
   cfg_byte_en_n              <= (OTHERS=>'1');
   cfg_wr_en_n                <= '1';
   cfg_rd_en_n                <= '1';
   cfg_dsn      <= X"00000001" &  X"01" & X"000A35";   -- //this is taken from GUI -

   cfg_turnoff_ok_n           <= '0';


   localId                    <= cfg_bus_number & cfg_device_number & cfg_function_number;

   pcie_link_width            <= cfg_lstatus(9 downto 4);



   trn_lnk_up_n_int_i: FDCP
     generic map (
                 INIT  => '1'
             )
     port map (
               Q     =>  trn_lnk_up_n,
               D     =>  trn_lnk_up_n_int1,
               C     =>  trn_clk,
               CLR   =>  '0',
               PRE   =>  '0'
              );


   trn_reset_n_i: FDCP
     generic map (
                 INIT  => '1'
              )
     port map (
               Q     =>  trn_reset_n,
               D     =>  trn_reset_n_int1,
               C     =>  trn_clk,
               CLR   =>  '0',
               PRE   =>  '0'
              );


-- --------------------------------------------------------------
-- --------------------------------------------------------------

make4Lanes: if pcieLanes = 4 generate
--S pcieCore : v6_pcie_v1_7_x1
    pcieCore : v6_pcie_v1_7_x4
    generic map (
                 PL_FAST_TRAIN  => FALSE
             )
    port map (

      ---------------------------------------------------------
      -- 1. PCI Express (pci_exp) Interface
      ---------------------------------------------------------

      -- Tx
      pci_exp_txp       =>     pci_exp_txp           ,
      pci_exp_txn       =>     pci_exp_txn           ,

      -- Rx
      pci_exp_rxp       =>     pci_exp_rxp           ,
      pci_exp_rxn       =>     pci_exp_rxn           ,
                                             
      ---------------------------------------------------------
      -- 2. Transaction (TRN) Interface
      ---------------------------------------------------------

      -- Common
      trn_clk           =>     trn_clk               ,
      trn_reset_n       =>     trn_reset_n_int1      ,
      trn_lnk_up_n      =>     trn_lnk_up_n_int1     ,

      -- Tx
      trn_tbuf_av       =>     trn_tbuf_av           ,
      trn_tcfg_req_n    =>     trn_tcfg_req_n        ,
      trn_terr_drop_n   =>     trn_terr_drop_n       ,
      trn_tdst_rdy_n    =>     trn_tdst_rdy_n        ,
      trn_td            =>     trn_td                ,
      trn_trem_n        =>     trn_trem_n(0)         ,
      trn_tsof_n        =>     trn_tsof_n            ,
      trn_teof_n        =>     trn_teof_n            ,
      trn_tsrc_rdy_n    =>     trn_tsrc_rdy_n        ,
      trn_tsrc_dsc_n    =>     trn_tsrc_dsc_n        ,
      trn_terrfwd_n     =>     trn_terrfwd_n         ,
      trn_tcfg_gnt_n    =>     trn_tcfg_gnt_n        ,
      trn_tstr_n        =>     trn_tstr_n            ,

      -- Rx
      trn_rd            =>     trn_rd                ,
      trn_rrem_n        =>     trn_rrem_n(0)         ,
      trn_rsof_n        =>     trn_rsof_n            ,
      trn_reof_n        =>     trn_reof_n            ,
      trn_rsrc_rdy_n    =>     trn_rsrc_rdy_n        ,
      trn_rsrc_dsc_n    =>     trn_rsrc_dsc_n        ,
      trn_rerrfwd_n     =>     trn_rerrfwd_n         ,
      trn_rbar_hit_n    =>     trn_rbar_hit_n        ,
      trn_rdst_rdy_n    =>     trn_rdst_rdy_n        ,
      trn_rnp_ok_n      =>     trn_rnp_ok_n          ,

      -- Flow Control
      trn_fc_cpld       =>     trn_fc_cpld           ,
      trn_fc_cplh       =>     trn_fc_cplh           ,
      trn_fc_npd        =>     trn_fc_npd            ,
      trn_fc_nph        =>     trn_fc_nph            ,
      trn_fc_pd         =>     trn_fc_pd             ,
      trn_fc_ph         =>     trn_fc_ph             ,
      trn_fc_sel        =>     trn_fc_sel            ,


      ---------------------------------------------------------
      -- 3. Configuration (CFG) Interface
      ---------------------------------------------------------

      cfg_do                   =>     cfg_do                     ,
      cfg_rd_wr_done_n         =>     cfg_rd_wr_done_n           ,
      cfg_di                   =>     cfg_di                     ,
      cfg_byte_en_n            =>     cfg_byte_en_n              ,
      cfg_dwaddr               =>     cfg_dwaddr                 ,
      cfg_wr_en_n              =>     cfg_wr_en_n                ,
      cfg_rd_en_n              =>     cfg_rd_en_n                ,

      cfg_err_cor_n            =>     cfg_err_cor_n              ,
      cfg_err_ur_n             =>     cfg_err_ur_n               ,
      cfg_err_ecrc_n           =>     cfg_err_ecrc_n             ,
      cfg_err_cpl_timeout_n    =>     cfg_err_cpl_timeout_n      ,
      cfg_err_cpl_abort_n      =>     cfg_err_cpl_abort_n        ,
      cfg_err_cpl_unexpect_n   =>     cfg_err_cpl_unexpect_n     ,
      cfg_err_posted_n         =>     cfg_err_posted_n           ,
      cfg_err_locked_n         =>     cfg_err_locked_n           ,
      cfg_err_tlp_cpl_header   =>     cfg_err_tlp_cpl_header     ,
      cfg_err_cpl_rdy_n        =>     cfg_err_cpl_rdy_n          ,
      cfg_interrupt_n          =>     cfg_interrupt_n            ,
      cfg_interrupt_rdy_n      =>     cfg_interrupt_rdy_n        ,
      cfg_interrupt_assert_n   =>     cfg_interrupt_assert_n     ,
      cfg_interrupt_di         =>     cfg_interrupt_di           ,
      cfg_interrupt_do         =>     cfg_interrupt_do           ,
      cfg_interrupt_mmenable   =>     cfg_interrupt_mmenable     ,
      cfg_interrupt_msienable  =>     cfg_interrupt_msienable    ,
      cfg_interrupt_msixenable =>     cfg_interrupt_msixenable   ,
      cfg_interrupt_msixfm     =>     cfg_interrupt_msixfm       ,
      cfg_turnoff_ok_n         =>     cfg_turnoff_ok_n           ,
      cfg_to_turnoff_n         =>     cfg_to_turnoff_n           ,
      cfg_trn_pending_n        =>     cfg_trn_pending_n          ,
      cfg_pm_wake_n            =>     cfg_pm_wake_n              ,
      cfg_bus_number           =>     cfg_bus_number             ,
      cfg_device_number        =>     cfg_device_number          ,
      cfg_function_number      =>     cfg_function_number        ,
      cfg_status               =>     cfg_status                 ,
      cfg_command              =>     cfg_command                ,
      cfg_dstatus              =>     cfg_dstatus                ,
      cfg_dcommand             =>     cfg_dcommand               ,
      cfg_lstatus              =>     cfg_lstatus                ,
      cfg_lcommand             =>     cfg_lcommand               ,
      cfg_dcommand2            =>     cfg_dcommand2              ,
      cfg_pcie_link_state_n    =>     cfg_pcie_link_state_n      ,
      cfg_dsn                  =>     cfg_dsn                    ,

      ---------------------------------------------------------
      -- 4. Physical Layer Control and Status (PL) Interface
      ---------------------------------------------------------

      pl_initial_link_width            =>     pl_initial_link_width           ,
      pl_lane_reversal_mode            =>     pl_lane_reversal_mode           ,
      pl_link_gen2_capable             =>     pl_link_gen2_capable            ,
      pl_link_partner_gen2_supported   =>     pl_link_partner_gen2_supported  ,
      pl_link_upcfg_capable            =>     pl_link_upcfg_capable           ,
      pl_ltssm_state                   =>     pl_ltssm_state                  ,
      pl_received_hot_rst              =>     pl_received_hot_rst             ,
      pl_sel_link_rate                 =>     pl_sel_link_rate                ,
      pl_sel_link_width                =>     pl_sel_link_width               ,
      pl_directed_link_auton           =>     pl_directed_link_auton          ,
      pl_directed_link_change          =>     pl_directed_link_change         ,
      pl_directed_link_speed           =>     pl_directed_link_speed          ,
      pl_directed_link_width           =>     pl_directed_link_width          ,
      pl_upstream_prefer_deemph        =>     pl_upstream_prefer_deemph       ,

      ---------------------------------------------------------
      -- 5. System  (SYS) Interface
      ---------------------------------------------------------

      sys_clk                          =>     sys_clk_c     ,
      sys_reset_n                      =>     sys_reset_n_c 
               
);

 end generate;


-- ---------------------------------------------------------------
-- tlp control module
-- ---------------------------------------------------------------

   trn_rrem_n(7 downto 1) <= X"0" & trn_rrem_n(0) & trn_rrem_n(0) & trn_rrem_n(0);

   theTlpControl:
   tlpControl 
   port map (

           mbuf_UserFull               => '0'                 ,
           trn_Blinker                 => trn_Blinker         ,

           -- Interrupter triggers
           DAQ_irq                     =>  DAQ_irq            ,  -- IN  std_logic;
           CTL_irq                     =>  CTL_irq            ,  -- IN  std_logic;
           DLM_irq                     =>  DLM_irq            ,  -- IN  std_logic;


--S	SIMONE: Wanxau UserLogic Signals, not Used
           -- DCB protocol interface
           protocol_link_act           =>  protocol_link_act  ,  -- IN  std_logic_vector(2-1 downto 0);
           protocol_rst                =>  protocol_rst       ,  -- OUT std_logic;
           Link_Buf_Full               =>  daq_rstop          ,  -- IN  std_logic;
           -- Fabric side: CTL Rx
           ctl_rv                      =>  ctl_rv             ,  -- OUT std_logic;
           ctl_rd                      =>  ctl_rd             ,  -- OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
           -- Fabric side: CTL Tx
           ctl_ttake                   =>  ctl_ttake          ,  -- OUT std_logic;
           ctl_tv                      =>  ctl_tv             ,  -- IN  std_logic;
           ctl_td                      =>  ctl_td             ,  -- IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
           ctl_tstop                   =>  ctl_tstop          ,  -- OUT std_logic;
           ctl_reset                   =>  ctl_reset          ,  -- OUT std_logic;
           ctl_status                  =>  ctl_status         ,  -- IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
           -- Fabric side: DLM Rx
           dlm_rv                      =>  dlm_rv             ,  -- OUT std_logic;
           dlm_rd                      =>  dlm_rd             ,  -- OUT std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
           -- Fabric side: DLM Tx
           dlm_tv                      =>  dlm_tv             ,  -- IN  std_logic;
           dlm_td                      =>  dlm_td             ,  -- IN  std_logic_vector(C_DBUS_WIDTH/2-1 downto 0);
           tab_we                      =>  tab_we             ,  -- OUT std_logic_vector(2-1 downto 0);
           tab_wa                      =>  tab_wa             ,  -- OUT std_logic_vector(12-1 downto 0);
           tab_wd                      =>  tab_wd             ,  -- OUT std_logic_vector(C_DBUS_WIDTH-1 downto 0);
           DG_is_Running               =>  dg_running         ,  -- IN  std_logic;
           DG_Reset                    =>  dg_rst             ,  -- OUT   STD_LOGIC;
           DG_Mask                     =>  dg_mask            ,  -- OUT   STD_LOGIC
--S	SIMONE: Wanxau UserLogic Signals, not Used


           -- SIMONE Register: PC-->FPGA
           reg01_tv                     => reg01_tv, 
           reg01_td                     => reg01_td,            
           reg02_tv                     => reg02_tv,            
           reg02_td                     => reg02_td,            
           reg03_tv                     => reg03_tv,            
           reg03_td                     => reg03_td,            
           reg04_tv                     => reg04_tv,            
           reg04_td                     => reg04_td,            
           reg05_tv                     => reg05_tv,            
           reg05_td                     => reg05_td,            
           reg06_tv                     => reg06_tv,            
           reg06_td                     => reg06_td,            
           reg07_tv                     => reg07_tv,            
           reg07_td                     => reg07_td,            
           reg08_tv                     => reg08_tv,            
           reg08_td                     => reg08_td,            
           reg09_tv                     => reg09_tv,            
           reg09_td                     => reg09_td,            
           reg10_tv                     => reg10_tv,            
           reg10_td                     => reg10_td,            
           reg11_tv                     => reg11_tv,            
           reg11_td                     => reg11_td,            
           reg12_tv                     => reg12_tv,            
           reg12_td                     => reg12_td,            
           reg13_tv                     => reg13_tv,            
           reg13_td                     => reg13_td,            
           reg14_tv                     => reg14_tv,            
           reg14_td                     => reg14_td,            

           -- SIMONE Register: FPGA-->PC
           reg01_rv                     => reg01_rv,            
           reg01_rd                     => reg01_rd,            
           reg02_rv                     => reg02_rv,            
           reg02_rd                     => reg02_rd,            
           reg03_rv                     => reg03_rv,            
           reg03_rd                     => reg03_rd,            
           reg04_rv                     => reg04_rv,            
           reg04_rd                     => reg04_rd,            
           reg05_rv                     => reg05_rv,            
           reg05_rd                     => reg05_rd,            
           reg06_rv                     => reg06_rv,            
           reg06_rd                     => reg06_rd,            
           reg07_rv                     => reg07_rv,            
           reg07_rd                     => reg07_rd,            
           reg08_rv                     => reg08_rv,            
           reg08_rd                     => reg08_rd,            
           reg09_rv                     => reg09_rv,            
           reg09_rd                     => reg09_rd,            
           reg10_rv                     => reg10_rv,            
           reg10_rd                     => reg10_rd,            
           reg11_rv                     => reg11_rv,            
           reg11_rd                     => reg11_rd,            
           reg12_rv                     => reg12_rv,            
           reg12_rd                     => reg12_rd,            
           reg13_rv                     => reg13_rv,            
           reg13_rd                     => reg13_rd,            
           reg14_rv                     => reg14_rv,            
           reg14_rd                     => reg14_rd,            

			  -- SIMONE debug signals	
			  debug_in_1i                  => debug_in_1i,	
			  debug_in_2i                  => debug_in_2i,	
			  debug_in_3i                  => debug_in_3i,	
			  debug_in_4i                  => debug_in_4i,	

           -- Event Buffer FIFO interface
           eb_FIFO_we                  => eb_we               , --  OUT std_logic; 
           eb_FIFO_wsof                => eb_wsof             , --  OUT std_logic; 
           eb_FIFO_weof                => eb_weof             , --  OUT std_logic; 
           eb_FIFO_din                 => eb_din(C_DBUS_WIDTH-1 downto 0) , --  OUT std_logic_vector(C_DBUS_WIDTH-1 downto 0);

           eb_FIFO_re                  => eb_re               , --  OUT std_logic; 
           eb_FIFO_empty               => eb_empty            , --  IN  std_logic; 
           eb_FIFO_qout                => eb_dout(C_DBUS_WIDTH-1 downto 0) , --  IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
           eb_FIFO_data_count          => eb_data_count       , --  IN  std_logic_vector(C_FIFO_DC_WIDTH downto 0);

           eb_FIFO_ow                  => eb_FIFO_ow          , --  IN  std_logic;

           pio_reading_status          => pio_reading_status  , --  OUT std_logic; 

           eb_FIFO_Status              => eb_FIFO_Status      , --  IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
           eb_FIFO_Rst                 => eb_rst              , --  OUT std_logic;
			  H2B_FIFO_Status					=> H2B_FIFO_Status     ,  						
			  B2H_FIFO_Status					=> B2H_FIFO_Status     ,  						

           -- Debugging signals
           DMA_us_Done                 => DMA_us_Done         , -- OUT std_logic;
           DMA_us_Busy                 => DMA_us_Busy         , -- OUT std_logic;
           DMA_us_Busy_LED             => LEDs_IO_pin(6)      , -- OUT std_logic;
           DMA_ds_Done                 => DMA_ds_Done         , -- OUT std_logic;
           DMA_ds_Busy                 => DMA_ds_Busy         , -- OUT std_logic;
           DMA_ds_Busy_LED             => LEDs_IO_pin(4)      , -- OUT std_logic;
                                                        
           -------------------
           -- DDR Interface
           DDR_Ready                   => DDR_Ready           , --  IN    std_logic;

           DDR_wr_sof                  => DDR_wr_sof          , --  OUT   std_logic;
           DDR_wr_eof                  => DDR_wr_eof          , --  OUT   std_logic;
           DDR_wr_v                    => DDR_wr_v            , --  OUT   std_logic;
           DDR_wr_FA                   => DDR_wr_FA           , --  OUT   std_logic;
           DDR_wr_Shift                => DDR_wr_Shift        , --  OUT   std_logic;
           DDR_wr_Mask                 => DDR_wr_Mask         , --  OUT   std_logic_vector(2-1 downto 0);
           DDR_wr_din                  => DDR_wr_din          , --  OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);
           DDR_wr_full                 => DDR_wr_full         , --  IN    std_logic;

           DDR_rdc_sof                 => DDR_rdc_sof         , --  OUT   std_logic;
           DDR_rdc_eof                 => DDR_rdc_eof         , --  OUT   std_logic;
           DDR_rdc_v                   => DDR_rdc_v           , --  OUT   std_logic;
           DDR_rdc_FA                  => DDR_rdc_FA          , --  OUT   std_logic;
           DDR_rdc_Shift               => DDR_rdc_Shift       , --  OUT   std_logic;
           DDR_rdc_din                 => DDR_rdc_din         , --  OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);
           DDR_rdc_full                => DDR_rdc_full        , --  IN    std_logic;

           -- DDR payload FIFO Read Port
           DDR_FIFO_RdEn               => DDR_FIFO_RdEn      ,  -- OUT std_logic; 
           DDR_FIFO_Empty              => DDR_FIFO_Empty     ,  -- IN  std_logic;
           DDR_FIFO_RdQout             => DDR_FIFO_RdQout    ,  -- IN  std_logic_vector(C_DBUS_WIDTH-1 downto 0);
                                                      
           -------------------
           -- Transaction Interface
           trn_lnk_up_n                => trn_lnk_up_n            ,
           trn_rsrc_dsc_n              => trn_rsrc_dsc_n          ,
           trn_rnp_ok_n                => trn_rnp_ok_n            ,
           trn_tsrc_dsc_n              => trn_tsrc_dsc_n          ,
           trn_tdst_dsc_n              => trn_tdst_dsc_n          ,
           trn_tbuf_av                 => trn_tbuf_av             ,
           trn_terrfwd_n               => trn_terrfwd_n           ,

           trn_clk                     => trn_clk                 ,
           trn_reset_n                 => trn_reset_n             ,
           trn_rsrc_rdy_n              => trn_rsrc_rdy_n          ,
           trn_tdst_rdy_n              => trn_tdst_rdy_n          ,
           trn_rsof_n                  => trn_rsof_n              ,
           trn_reof_n                  => trn_reof_n              ,
           trn_rerrfwd_n               => trn_rerrfwd_n           ,
           trn_rrem_n                  => trn_rrem_n              ,
           trn_rd                      => trn_rd                  ,
                                                        
           cfg_interrupt_n             => cfg_interrupt_n         ,
           cfg_interrupt_rdy_n         => cfg_interrupt_rdy_n     ,
           cfg_interrupt_mmenable      => cfg_interrupt_mmenable  ,
           cfg_interrupt_msienable     => cfg_interrupt_msienable ,
           cfg_interrupt_di            => cfg_interrupt_di        ,
           cfg_interrupt_do            => cfg_interrupt_do        ,
           cfg_interrupt_assert_n      => cfg_interrupt_assert_n  ,

           trn_rbar_hit_n              => trn_rbar_hit_n          ,
           trn_tsrc_rdy_n              => trn_tsrc_rdy_n          ,
           trn_rdst_rdy_n              => trn_rdst_rdy_n          ,
           trn_tsof_n                  => trn_tsof_n              ,
           trn_teof_n                  => trn_teof_n              ,
           trn_trem_n                  => trn_trem_n              ,
           trn_td                      => trn_td                  ,

           Format_Shower               => Format_Shower           ,

           cfg_dcommand                => cfg_dcommand            ,
           pcie_link_width             => pcie_link_width         ,
           localId                     => localId
           );
	

  -- -----------------------------------------------------------------------
  --  DDR SDRAM: control module USER LOGIC (2 BRAM Module: 
  -- -----------------------------------------------------------------------


   LoopBack_BRAM_Off:  if not USE_LOOPBACK_TEST generate

   DDRs_ctrl_module:
   bram_DDRs_Control
   GENERIC MAP (
                C_ASYNFIFO_WIDTH    => 72 ,
                P_SIMULATION        => FALSE
               )
   PORT MAP(

      user_wr_weA             =>  user_wr_weA   ,
      user_wr_addrA           =>  user_wr_addrA ,
      user_wr_dinA            =>  user_wr_dinA  ,
      user_rd_addrB           =>  user_rd_addrB ,
      user_rd_doutB           =>  user_rd_doutB ,
      user_rd_clk             =>  clk_200MHz    ,
      user_wr_clk             =>  clk_200MHz    ,
      -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
      DDR_wr_sof               => DDR_wr_sof          , --  IN    std_logic;
      DDR_wr_eof               => DDR_wr_eof          , --  IN    std_logic;
      DDR_wr_v                 => DDR_wr_v            , --  IN    std_logic;
      DDR_wr_FA                => DDR_wr_FA           , --  IN    std_logic;
      DDR_wr_Shift             => DDR_wr_Shift        , --  IN    std_logic;
      DDR_wr_Mask              => DDR_wr_Mask         , --  IN    std_logic_vector(2-1 downto 0);
      DDR_wr_din               => DDR_wr_din          , --  IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_wr_full              => DDR_wr_full         , --  OUT   std_logic;

      DDR_rdc_sof              => DDR_rdc_sof         , --  IN    std_logic;
      DDR_rdc_eof              => DDR_rdc_eof         , --  IN    std_logic;
      DDR_rdc_v                => DDR_rdc_v           , --  IN    std_logic;
      DDR_rdc_FA               => DDR_rdc_FA          , --  IN    std_logic;
      DDR_rdc_Shift            => DDR_rdc_Shift       , --  IN    std_logic;
      DDR_rdc_din              => DDR_rdc_din         , --  IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_rdc_full             => DDR_rdc_full        , --  OUT   std_logic;

      -- DDR payload FIFO Read Port
      DDR_FIFO_RdEn            => DDR_FIFO_RdEn       ,  -- IN    std_logic; 
      DDR_FIFO_Empty           => DDR_FIFO_Empty      ,  -- OUT   std_logic;
      DDR_FIFO_RdQout          => DDR_FIFO_RdQout     ,  -- OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);

      -- Common interface
      DDR_Ready                => DDR_Ready           , --  OUT   std_logic;
      DDR_Blinker              => DDR_Blinker         , --  OUT   std_logic;
      mem_clk                  => trn_clk             , --  IN
      trn_clk                  => trn_clk             , --  IN    std_logic;
		Sim_Zeichen              => Sim_Zeichen         , --  OUT   std_logic;  
      trn_reset_n              => trn_reset_n           --  IN    std_logic
    );
   
	end generate;



   LoopBack_BRAM_On:  if USE_LOOPBACK_TEST generate


   DDRs_ctrl_module:
   bram_DDRs_Control_loopback
   GENERIC MAP (
                C_ASYNFIFO_WIDTH    => 72 ,
                P_SIMULATION        => FALSE
               )
   PORT MAP(

      -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
      DDR_wr_sof               => DDR_wr_sof          , --  IN    std_logic;
      DDR_wr_eof               => DDR_wr_eof          , --  IN    std_logic;
      DDR_wr_v                 => DDR_wr_v            , --  IN    std_logic;
      DDR_wr_FA                => DDR_wr_FA           , --  IN    std_logic;
      DDR_wr_Shift             => DDR_wr_Shift        , --  IN    std_logic;
      DDR_wr_Mask              => DDR_wr_Mask         , --  IN    std_logic_vector(2-1 downto 0);
      DDR_wr_din               => DDR_wr_din          , --  IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_wr_full              => DDR_wr_full         , --  OUT   std_logic;

      DDR_rdc_sof              => DDR_rdc_sof         , --  IN    std_logic;
      DDR_rdc_eof              => DDR_rdc_eof         , --  IN    std_logic;
      DDR_rdc_v                => DDR_rdc_v           , --  IN    std_logic;
      DDR_rdc_FA               => DDR_rdc_FA          , --  IN    std_logic;
      DDR_rdc_Shift            => DDR_rdc_Shift       , --  IN    std_logic;
      DDR_rdc_din              => DDR_rdc_din         , --  IN    std_logic_vector(C_DBUS_WIDTH-1 downto 0);
      DDR_rdc_full             => DDR_rdc_full        , --  OUT   std_logic;

      -- DDR payload FIFO Read Port
      DDR_FIFO_RdEn            => DDR_FIFO_RdEn       ,  -- IN    std_logic; 
      DDR_FIFO_Empty           => DDR_FIFO_Empty      ,  -- OUT   std_logic;
      DDR_FIFO_RdQout          => DDR_FIFO_RdQout     ,  -- OUT   std_logic_vector(C_DBUS_WIDTH-1 downto 0);

      -- Common interface
      DDR_Ready                => DDR_Ready           , --  OUT   std_logic;
      DDR_Blinker              => DDR_Blinker         , --  OUT   std_logic;
      mem_clk                  => trn_clk             , --  IN
      trn_clk                  => trn_clk             , --  IN    std_logic;
		Sim_Zeichen              => Sim_Zeichen         , --  OUT   std_logic;  
      trn_reset_n              => trn_reset_n           --  IN    std_logic
    );

   end generate;



    LEDs_IO_pin(0)    <= trn_reset_n xor Format_Shower;
    LEDs_IO_pin(1)    <= trn_lnk_up_n  ;
    LEDs_IO_pin(2)    <= Format_Shower ;
    LEDs_IO_pin(3)    <= trn_Blinker   ;



    ------------------------ -----------------------
    -- Event Buffer wrapper (FIFO Module: H2B & B2H)
    ------------------------ -----------------------

   LoopBack_FIFO_Off:  if not USE_LOOPBACK_TEST generate

    queue_buffer0:
    eb_wrapper
      port map (  

         H2B_wr_clk     		=> trn_clk   				,  
         H2B_wr_en      		=> eb_we  					,
         H2B_wr_din       		=> eb_din 					,
         H2B_wr_pfull     		=> eb_pfull  				,
         H2B_wr_full      		=> eb_full   				,
         H2B_wr_data_count    => H2B_wr_data_count(C_EMU_FIFO_DC_WIDTH-1+1 downto 1) ,

         H2B_rd_clk           => clk_200MHz       		,
			H2B_rd_en            => user_rd_en        	,
         H2B_rd_dout          => user_rd_dout      	,
         H2B_rd_pempty        => user_rd_pempty    	,
         H2B_rd_empty         => user_rd_empty     	,
         H2B_rd_valid         => user_rd_valid     	,
         H2B_rd_data_count    => user_rd_data_count	,
         
			B2H_wr_clk           => clk_200MHz       		,
         B2H_wr_en            => user_wr_en        	,
         B2H_wr_din           => user_wr_din       	,
         B2H_wr_pfull         => user_wr_pfull     	,
         B2H_wr_full          => user_wr_full      	,
         B2H_wr_data_count    => user_wr_data_count	,


         B2H_rd_clk        	=> trn_clk   				,  
         B2H_rd_en         	=> eb_re     				,
         B2H_rd_dout       	=> eb_dout   				,
         B2H_rd_pempty     	=> eb_pempty 				,
         B2H_rd_empty      	=> eb_empty  				,
         B2H_rd_valid      	=> eb_valid  				,
         B2H_rd_data_count 	=> B2H_rd_data_count(C_EMU_FIFO_DC_WIDTH-1+1 downto 1) ,

         rst        				=> eb_rst    
         );



		--- 64 bits to 32 bits transformation ( --> Count * 2)--- 
      B2H_rd_data_count(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1)
                        <= C_ALL_ZEROS(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1);
      B2H_rd_data_count(0) <= '0';       
                      
      H2B_wr_data_count(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1)
                        <= C_ALL_ZEROS(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1);
      H2B_wr_data_count(0) <= '0';       
							 

		--- Hybrid FIFO Signal used by PCIe interface and Linux Driver
      eb_FIFO_ow      <= eb_we_up and eb_full;
      fifo_reset_done <= not eb_rst;
      eb_din(72-1 downto C_DBUS_WIDTH) <= (OTHERS=>'0');
		eb_data_count   <= B2H_rd_data_count;

		--- Hybrid FIFO Status used by PCIe interface and Linux Driver ---
		--- read: status ; write: reset H2B and B2H FIFO
      eb_FIFO_Status(C_DBUS_WIDTH-1 downto C_FIFO_DC_WIDTH+3)
                           <= (OTHERS=>'0');
      eb_FIFO_Status(C_FIFO_DC_WIDTH+2 downto 3)
                           <= B2H_rd_data_count(C_FIFO_DC_WIDTH downto 1);
      eb_FIFO_Status(2)    <= '0';      
      eb_FIFO_Status(1)    <= eb_pfull;
      eb_FIFO_Status(0)    <= eb_empty and fifo_reset_done;


		--- Host2Board FIFO status used by user ---
		--- read: H2B status ; write: nothing 
      H2B_FIFO_Status(C_DBUS_WIDTH-1 downto C_FIFO_DC_WIDTH+3)
                           <= (OTHERS=>'0');
      H2B_FIFO_Status(C_FIFO_DC_WIDTH+2 downto 3)
                           <= H2B_wr_data_count(C_FIFO_DC_WIDTH downto 1);
      H2B_FIFO_Status(2)    <= '0';
      H2B_FIFO_Status(1)    <= eb_pfull;
      H2B_FIFO_Status(0)    <= eb_full and fifo_reset_done;


		--- Board2Host FIFO status used by user ---
		--- read: B2H status ; write: nothing 
      B2H_FIFO_Status(C_DBUS_WIDTH-1 downto C_FIFO_DC_WIDTH+3)
                           <= (OTHERS=>'0');
      B2H_FIFO_Status(C_FIFO_DC_WIDTH+2 downto 3)
                           <= B2H_rd_data_count(C_FIFO_DC_WIDTH downto 1);
      B2H_FIFO_Status(2)    <= eb_valid;
      B2H_FIFO_Status(1)    <= eb_pempty;
      B2H_FIFO_Status(0)    <= eb_empty and fifo_reset_done;


   end generate;
	
   LoopBack_FIFO_On:  if USE_LOOPBACK_TEST generate

     queue_buffer0:
     eb_wrapper_loopback
      port map (
         wr_clk     => trn_clk   ,  -- eb_wclk   ,
         wr_en      => eb_we  ,
         din        => eb_din ,
         pfull      => eb_pfull  ,
         full       => eb_full   ,

         rd_clk     => trn_clk   ,  -- eb_rclk   ,
         rd_en      => eb_re     ,
         dout       => eb_dout   ,
         pempty     => eb_pempty ,
         empty      => eb_empty  ,

         data_count => eb_data_count(C_EMU_FIFO_DC_WIDTH-1+1 downto 1) ,
         rst        => eb_rst    
         );

     eb_data_count(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1)
                      <= C_ALL_ZEROS(C_FIFO_DC_WIDTH downto C_EMU_FIFO_DC_WIDTH+1);
     eb_data_count(0)<= '0';
     fifo_reset_done <= not eb_rst;
     eb_FIFO_ow           <= eb_we_up and eb_full;
     eb_din(72-1 downto C_DBUS_WIDTH)       <= (OTHERS=>'0');

     eb_FIFO_Status(C_DBUS_WIDTH-1 downto C_FIFO_DC_WIDTH+3)
                         <= (OTHERS=>'0');
     eb_FIFO_Status(C_FIFO_DC_WIDTH+2 downto 3)
                         <= eb_data_count(C_FIFO_DC_WIDTH downto 1);
     eb_FIFO_Status(2)    <= '0'; 
     eb_FIFO_Status(1)    <= eb_pfull;
     eb_FIFO_Status(0)    <= eb_empty and fifo_reset_done;
	 
	 
     H2B_FIFO_Status <= (OTHERS=>'0');	 
	  H2B_FIFO_Status <= (OTHERS=>'0');	  	
		
   end generate;


end Behavioral;
