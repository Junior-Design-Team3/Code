----------------------------------------------------------------------------------
-- Company: 	Binghamton University
-- Engineer: 	Walter Keyes
-- 
-- Create Date:    15:54:54 05/04/2018 
-- Design Name: 
-- Module Name:    Metal_Detector - Behavioral 
-- Project Name: 	 EECE 387 Final Project
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
	use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Metal_Detector is
    Port ( clk : in  STD_LOGIC;
			  rst : in	STD_LOGIC;
           pulse_in : in  STD_LOGIC;
           detect_in : in  STD_LOGIC;
           detect_out : out  STD_LOGIC);
end Metal_Detector;

architecture Behavioral of Metal_Detector is

	constant THRESHOLD : unsigned(5 downto 0) := "110010";
	signal count : unsigned(5 downto 0);
	signal pulse_d1 : std_logic;
	signal detect_d1 : std_logic;
	signal pulse_rise : std_logic;
	signal detect_rise : std_logic;

	type state_type is ( st1_idle, st2_pulse, st3_detected, st4_no_pulse);
	signal state, next_state : state_type := st1_idle;

	signal en_count : std_logic;
	signal count_clr : std_logic;
	signal detect_out_i : std_logic;

begin
	process(clk)
	begin
		if rising_edge(clk) then
			pulse_d1 <= pulse_in;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			detect_d1 <= detect_in;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if detect_in = '1' and detect_d1 = '0' then
				detect_rise <= '1';
			else
				detect_rise <= '0';
			end if;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if pulse_in = '1' and pulse_d1 = '0' then
				pulse_rise <= '1';
			else
				pulse_rise <= '0';
			end if;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if count_clr = '1' then
				count <= (others => '0');
			elsif en_count = '1' and pulse_rise = '1' then
				count <= count + 1;
			else
				count <= count;
			end if;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				state <= st1_idle;
			else
				state <= next_state;
			end if;
		end if;
	end process;
	
	process(state, detect_rise, detect_in, pulse_rise, count)
	begin
		en_count <= '0';
		count_clr <= '0';
		detect_out_i <= '0';
		next_state <= state;
		
		case state is
			when st1_idle =>
				count_clr <= '1';
				if detect_rise = '1' then
					next_state <= st2_pulse;
				end if;
			when st2_pulse =>
				en_count <= '1';
				if pulse_rise = '1' and detect_in = '0' then
					next_state <= st1_idle;
				elsif pulse_rise = '1' and detect_in = '1' and count = THRESHOLD then
					next_state <= st3_detected;
				end if;
			when st3_detected =>
				count_clr <= '1';
				detect_out_i <= '1';
				if pulse_rise = '1' and detect_in = '0' then
					next_state <= st4_no_pulse;
				end if;
			when st4_no_pulse =>
				en_count <= '1';
				detect_out_i <= '1';
				if pulse_rise = '1' and detect_in = '0' and count = THRESHOLD then
					next_state <= st1_idle;
				elsif pulse_rise = '1' and detect_in = '1' then
					next_state <= st3_detected;
				end if;
		end case;
	end process;
	
	detect_out <= detect_out_i;


end Behavioral;

