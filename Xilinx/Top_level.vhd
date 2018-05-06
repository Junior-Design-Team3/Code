----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    17:50:09 05/03/2018 
-- Design Name: 
-- Module Name:    Top_level - Behavioral 
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Top_level is
    Port ( clk : in  STD_LOGIC;
			  rst : in STD_LOGIC;
			  resume_mine : in STD_LOGIC;
			  detector_in : in STD_LOGIC;
           detector_pulse : out  STD_LOGIC;
			  detector_out : out STD_LOGIC;
			  encoder_1 : in STD_LOGIC;
			  encoder_2 : in STD_LOGIC;
			  LED : out STD_LOGIC_VECTOR(7 downto 0);
			  Seg7_SEG : out STD_LOGIC_VECTOR(6 downto 0);
			  Seg7_AN : out STD_LOGIC_VECTOR(3 downto 0));
end Top_level;

architecture Behavioral of Top_level is
	
	COMPONENT CLK_DIV
	PORT(
		clk : IN std_logic;
		rst : IN std_logic;          
		clk_out : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT Metal_Detector
	PORT(
		clk : IN std_logic;
		rst : IN std_logic;
		pulse_in : IN std_logic;
		detect_in : IN std_logic;          
		detect_out : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT Enc_Dist
	PORT(
		clk : IN std_logic;
		rst : IN std_logic;
		enc_1 : IN std_logic;
		enc_2 : IN std_logic;          
		distance : OUT std_logic_vector(15 downto 0)
		);
	END COMPONENT;
	
	COMPONENT HEXon7segDisp
	PORT(
		hex_data_in0 : IN std_logic_vector(3 downto 0);
		hex_data_in1 : IN std_logic_vector(3 downto 0);
		hex_data_in2 : IN std_logic_vector(3 downto 0);
		hex_data_in3 : IN std_logic_vector(3 downto 0);
		dp_in : IN std_logic_vector(2 downto 0);
		clk : IN std_logic;          
		seg_out : OUT std_logic_vector(6 downto 0);
		an_out : OUT std_logic_vector(3 downto 0);
		dp_out : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT LED_mines
	PORT(
		clk : IN std_logic;
		rst : IN std_logic;
		mine_det : IN std_logic;
		resume_cnt : IN std_logic;          
		led_out : OUT std_logic_vector(7 downto 0)
		);
	END COMPONENT;

	signal detector_clk : std_logic;
	signal enc_distance : std_logic_vector(15 downto 0);
	signal mine_detected : std_logic;
	signal resume_detection : std_logic;
	signal LED_out : std_logic_vector(7 downto 0);
	
begin

	detector_pulse <= detector_clk;
	LED <= LED_out;
	resume_detection <= resume_mine;
	
	
	Inst_CLK_DIV: CLK_DIV PORT MAP(
		clk => clk,
		rst => rst,
		clk_out => detector_clk
	);
	
	Inst_Metal_Detector: Metal_Detector PORT MAP(
		clk => clk,
		rst => rst,
		pulse_in => detector_clk,
		detect_in => detector_in,
		detect_out => mine_detected
	);
	
	Inst_Enc_Dist: Enc_Dist PORT MAP(
		clk => clk,
		rst => rst,
		enc_1 => encoder_1,
		enc_2 => encoder_2,
		distance => enc_distance
	);
	
		Inst_HEXon7segDisp: HEXon7segDisp PORT MAP(
		hex_data_in3 => enc_distance(3 downto 0),
		hex_data_in2 => enc_distance(7 downto 4),
		hex_data_in1 => enc_distance(11 downto 8),
		hex_data_in0 => enc_distance(15 downto 12),
		dp_in => (others => '0'),
		seg_out => Seg7_SEG,
		an_out => Seg7_AN,
		dp_out => open,
		clk => clk
	);
	
	Inst_LED_mines: LED_mines PORT MAP(
		clk => clk,
		rst => rst,
		mine_det => mine_detected,
		resume_cnt => resume_detection,
		led_out => LED_out
	);
	
	detector_out <= mine_detected;

end Behavioral;

