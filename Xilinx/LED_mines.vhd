----------------------------------------------------------------------------------
-- Company: Binghamton University
-- Engineer: Walter Keyes
-- 
-- Create Date:    11:40:08 05/05/2018 
-- Design Name: 
-- Module Name:    LED_mines - Behavioral 
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity LED_mines is
    Port ( clk : in  STD_LOGIC;
           rst : in  STD_LOGIC;
           mine_det : in  STD_LOGIC;
           resume_cnt : in  STD_LOGIC;
           led_out : out  STD_LOGIC_VECTOR (7 downto 0));
end LED_mines;

architecture Behavioral of LED_mines is

signal led_out_i : std_logic_vector(7 downto 0) := (others => '0');
signal en_shift : std_logic;
signal mine_det_d1 : std_logic;
signal resume_cnt_d1 : std_logic;
signal mine_det_rise : std_logic;
signal resume_cnt_rise : std_logic;

type state_type is (st1_idle, st2_wait);
signal state, next_state : state_type := st1_idle;

begin

	led_out <= led_out_i;

	process(clk)
	begin
		if rising_edge(clk) then
			mine_det_d1 <= mine_det;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			resume_cnt_d1 <= resume_cnt;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if resume_cnt = '1' and resume_cnt_d1 = '0' then
				resume_cnt_rise <= '1';
			else
				resume_cnt_rise <= '0';
			end if;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if mine_det = '1' and mine_det_d1 = '0' then
				mine_det_rise <= '1';
			else
				mine_det_rise <= '0';
			end if;
		end if;
	end process;

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				led_out_i <= (others => '0');
			elsif en_shift = '1' and mine_det_rise = '1' then
				led_out_i <= led_out_i(6 downto 0) & '1';
			else
				led_out_i <= led_out_i;
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
	
	process(state, mine_det_rise, resume_cnt_rise)
	begin
		en_shift <= '0';
		next_state <= state;
		
		case state is
			when st1_idle =>
				en_shift <= '1';
				if mine_det_rise = '1' then
					next_state <= st2_wait;
				end if;
			when st2_wait =>
				if resume_cnt_rise = '1' then
					next_state <= st1_idle;
				end if;
		end case;
	end process;


end Behavioral;

