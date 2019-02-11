-------------------------------------------------------------------------------
-- Title      : CAN BFM testbench
-- Project    :
-------------------------------------------------------------------------------
-- File       : can_bfm_base_tb.vhd
-- Author     : Simon Voigt Nesbø  <svn@hvl.no>
-- Company    :
-- Created    : 2019-02-11
-- Last update: 2019-02-11
-- Platform   :
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: Simple testbench of CAN bus BFM (base VHDL version, no UVVM).
--              The testbench simply transmits 1000 packages using the
--              can_write procedure, and receives them with can_read.
--              A simple check is done to see if the received data matches
--              what was transmitted.
-------------------------------------------------------------------------------
-- Copyright (c) 2019
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2019-02-11  1.0      simon	Created
-------------------------------------------------------------------------------



library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.can_bfm_base_pkg.all;


entity can_bfm_base_tb is
end entity can_bfm_base_tb;

architecture tb of can_bfm_base_tb is
  constant C_PACKAGE_COUNT : natural   := 1000;
  signal clk               : std_logic := '0';
  signal can_bus_signal    : std_logic;

  signal can_tx1 : std_logic := '1';
  signal can_rx1 : std_logic := '1';

  signal can_tx2 : std_logic := '1';
  signal can_rx2 : std_logic := '1';

  signal arb_id_xmit : std_logic_vector(28 downto 0);
  signal data_xmit   : can_payload_t;

  signal data_length_xmit : natural;

begin  -- architecture tb

  -- "Tristating" of canbus line
  can_bus_signal <= 'H';
  can_bus_signal <= '0' when can_tx1 = '0' else 'Z';
  can_bus_signal <= '0' when can_tx2 = '0' else 'Z';
  can_rx1        <= '1' ?= can_bus_signal;
  can_rx2        <= '1' ?= can_bus_signal;


  proc_tb_transmit: process is
    variable xmit_count        : natural   := 0;
    variable seed1             : positive  := 12345;
    variable seed2             : positive  := 6789;
    variable rand_real         : real;
    variable rand_size         : natural;
    variable rand_byte         : natural;
    variable rand_id           : natural;
    variable can_bit_stuffing1 : std_logic := '0';
    variable can_sample_point1 : std_logic := '0';
    variable arb_lost          : std_logic := '0';
    variable ack_received      : std_logic := '0';
  begin
    wait for 6 us;

    -- Send random CAN frames
    for xmit_count in 0 to C_PACKAGE_COUNT-1 loop
      wait for 15 us;

      -- Generate random frame
      uniform(seed1, seed2, rand_real);
      rand_size        := natural(round(rand_real * real(8)));
      data_length_xmit <= rand_size;

      uniform(seed1, seed2, rand_real);
      rand_id                  := natural(round(rand_real * real(2**29-1)));
      arb_id_xmit(28 downto 0) <= std_logic_vector(to_unsigned(rand_id, 29));

      for byte_num in 0 to 7 loop
        if byte_num < rand_size then
          uniform(seed1, seed2, rand_real);
          rand_byte           := natural(round(rand_real * real(255)));
          data_xmit(byte_num) <= std_logic_vector(to_unsigned(rand_byte, 8));
        else
          data_xmit(byte_num) <= x"00";
        end if;
      end loop;  -- byte_num

      -- Wait for a clock cycle to allow signals to update
      -- before calling can_write()
      wait until rising_edge(clk);

      can_write(arb_id_xmit,
                '0',                    -- Remote request off
                '1',                    -- Extended ID on
                data_xmit,
                rand_size,
                clk,
                can_tx1,
                can_rx1,
                can_bit_stuffing1,
                can_sample_point1,
                arb_lost,
                ack_received,
                '1',                    -- Bit stuffing enabled
                20 ns);
    end loop;  -- xmit_count

    report "Finished transmitting " & integer'image(C_PACKAGE_COUNT) & " packages.";

    wait;

  end process proc_tb_transmit;

  proc_tb_receive: process is
    variable recv_count        : natural   := 0;
    variable can_bit_stuffing2 : std_logic;
    variable can_sample_point2 : std_logic;
    variable arb_id_recv       : std_logic_vector(28 downto 0);
    variable remote_frame_recv : std_logic;
    variable extended_id_recv  : std_logic;
    variable data_recv         : can_payload_t;
    variable data_length_recv  : natural;
    variable timeout_recv      : std_logic;
    variable crc_error_recv    : std_logic;
  begin
    wait for 5 us;

    for recv_count in 0 to C_PACKAGE_COUNT-1 loop
      wait for 10 us;
      can_read(
        arb_id_recv,
        remote_frame_recv,
        extended_id_recv,
        data_recv,
        data_length_recv,
        100000,
        clk,
        can_rx2,
        can_tx2,
        can_bit_stuffing2,
        can_sample_point2,
        '1',
        timeout_recv,
        crc_error_recv,
        20 ns);

      wait until rising_edge(clk);

      assert data_length_recv = data_length_xmit report "TB: Received data length did not match transmit length" severity failure;

      assert data_recv(0 to data_length_recv-1) = data_xmit(0 to data_length_recv-1)
        report "TB: Received data did not match transmitted data" severity failure;

      wait until rising_edge(clk);

      --wait for 1 us;
    end loop;

    report "Finished receiving " & integer'image(C_PACKAGE_COUNT) & " packages.";

    std.env.finish;
  end process proc_tb_receive;

  proc_clk: process is
  begin  -- process clk_proc
    clk <= '0';
    wait for 10 ns;
    clk <= '1';
    wait for 10 ns;
  end process proc_clk;

end architecture tb;
