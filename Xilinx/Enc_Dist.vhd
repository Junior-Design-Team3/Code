----------------------------------------------------------------------------------
-- Company: Bingahmton University
-- Engineer: Walter Keyes
-- 
-- Create Date:    15:27:46 05/04/2018 
-- Design Name: 
-- Module Name:    Enc_Dist - Behavioral 
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

entity Enc_Dist is
    Port ( clk : in  STD_LOGIC;
			  rst : in	STD_LOGIC;
           enc_1 : in  STD_LOGIC;
           enc_2 : in  STD_LOGIC;
           distance : out  STD_LOGIC_VECTOR (15 downto 0));
end Enc_Dist;

architecture Behavioral of Enc_Dist is

	COMPONENT Pulser
	PORT(
		clk : IN std_logic;
		sig_in : IN std_logic;          
		sig_out : OUT std_logic
		);
	END COMPONENT;
	
	constant DIV : unsigned(4 downto 0) := "00001";
	
	signal dist_i : unsigned(15 downto 0) := (others => '0');
	signal inc : std_logic;
	signal enc : std_logic;

begin

	Inst_Pulser: Pulser PORT MAP(
		clk => clk,
		sig_in => enc,
		sig_out => inc
	);

	enc <= enc_1 xor enc_2;
	
	process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				dist_i <= (others => '0');
			elsif inc = '1' then
				dist_i <= dist_i + 1;
			else
				dist_i <= dist_i;
			end if;
		end if;
	end process;
	
	distance <= std_logic_vector(dist_i / DIV);
	
end Behavioral;

