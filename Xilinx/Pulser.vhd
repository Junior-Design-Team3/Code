----------------------------------------------------------------------------------
-- Company: 	Binghamton University
-- Engineer: 	Walter Keyes
-- 
-- Create Date:    15:38:25 05/04/2018 
-- Design Name: 
-- Module Name:    Pulser - Behavioral 
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

entity Pulser is
    Port ( clk : in  STD_LOGIC;
           sig_in : in  STD_LOGIC;
           sig_out : out  STD_LOGIC);
end Pulser;

architecture Behavioral of Pulser is
	signal sig_in_d1 : std_logic;
begin

	process(clk)
	begin
		if rising_edge(clk) then
			sig_in_d1 <= sig_in;
		end if;
	end process;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if sig_in = '1' and sig_in_d1 = '0' then
				sig_out <= '1';
			else
				sig_out <= '0';
			end if;
		end if;
	end process;
	

end Behavioral;

