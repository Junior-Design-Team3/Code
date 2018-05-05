----------------------------------------------------------------------------------
-- Company: Binghamton University
-- Engineer: Walter Keyes
-- 
-- Create Date:    17:22:46 05/03/2018 
-- Design Name: 	 
-- Module Name:    CLK_DIV - Behavioral 
-- Project Name: 	 EECE 387 Final Project
-- Target Devices: Papilio Duo
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

entity CLK_DIV is
    Port ( clk : in  STD_LOGIC;
			  rst : in	STD_LOGIC;
           clk_out : out  STD_LOGIC);
end CLK_DIV;

architecture Behavioral of CLK_DIV is
	constant CLK_MAX : unsigned(14 downto 0) := "111110011111111";
	constant CLK_THRESHOLD : unsigned(14 downto 0) := "001100100000000";
	signal counter : unsigned(14 downto 0) := CLK_MAX;
begin

	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' or counter = 0 then
				counter <= CLK_MAX;
			else
				counter <= counter - 1;
			end if;
		end if;
	end process;
	
	process(counter)
	begin
		if counter < CLK_THRESHOLD then
			clk_out <= '1';
		else
			clk_out <= '0';
		end if;
	end process;

end Behavioral;

