import cocotb
from cocotb_bus.bus import Bus
from cocotb.binary import BinaryValue
from cocotb.triggers import Timer, RisingEdge, First, ReadOnly, ClockCycles
from random import Random

import logging
from cocotb.log import SimLog

from abc import ABC, abstractmethod

class MemInputFrame():
    """
    A convenience class for setting the data signals on a MemInputBus
    """
    def __init__(self, wr_en=0, rd_en=0, addr=0, wr_data=b'', byte_en=0, size=None):
        self.wr_en = wr_en
        self.rd_en = rd_en
        self.addr = addr
        self.wr_data = wr_data
        self.byte_en = byte_en
        self.size = size

    def __repr__(self):
        return f"wr_en={self.wr_en}, rd_en={self.rd_en}, addr={self.addr}, " \
               f"wr_data={self.wr_data.hex()}, byte_en={self.byte_en}, size={self.size}"

"""
    A wrapper for a bus to be used as the input to a DRAM

    entity: whatever Bus expects. Probably your dut

    signals: a dictionary mapping signalNames to signal objects. You must have
    all the signals defined in _signalNames
"""
class MemInputBus(Bus):
    _signalNames = ["wr_en", "rd_en", "addr", "wr_data", "byte_en", "rdy"]
    def __init__(self, entity, signals):
        for name in self._signalNames:
            if not name in signals:
                raise AttributeError(f"signals doesn't contain a value for key" \
                    f"{name}")
        super().__init__(entity, "", signals, case_insensitive=False)
    
    def get_signals(self):
        return_vals = MemInputFrame()
        for attr_name in self._signals:
            val = getattr(self, attr_name).value
            setattr(return_vals, attr_name, val)

        return return_vals


"""
    A wrapper for a bus to be used as the output to a DRAM

    entity: whatever Bus expects. Probably your dut

    signals: a dictionary mapping signalNames to signal objects. You must have
    all the signals defined in _signalNames
"""
class MemOutputBus(Bus):
    _signalNames = ["rd_val", "rd_data", "rd_rdy"]
    def __init__(self, entity, signals):
        for name in self._signalNames:
            if not name in signals:
                raise AttributeError(f"signals doesn't contain a value for key" \
                    f"{name}")
        super().__init__(entity, "", signals, case_insensitive=False)

class FakeMem():
    def __init__(self, in_bus, out_bus, clk, mem_width=512, mem_depth=1024, input_delay_gen=None,
            op_delay_gen=None, use_byte_addr=False):
        self.in_bus = in_bus
        self.out_bus = out_bus
        self.clk = clk
        self.log = SimLog("cocotb.fake_mem")
        self.log.setLevel(logging.INFO)
        self.mem_width = mem_width
        self.mem_bytes = int(mem_width/8)
        self.mem = [bytearray([0] * self.mem_bytes) for i in range(mem_depth)]
        self.use_byte_addr = use_byte_addr

        self.input_delay_gen = input_delay_gen
        self.op_delay_gen = op_delay_gen

    def __repr__(self):
        title_str = f"FakeMem(mem_width={self.mem_width}, mem_depth={len(self.mem)})\n"
        for line in self.mem:
            line_str = " ".join(f"{byte:02x}" for byte in line)
            title_str += f"{line_str}\n"
        return title_str

    def read_mem(self, address, length):
        block_addr_width = (self.mem_bytes - 1).bit_length()
        block_addr_mask = (1 << block_addr_width) - 1

        res_buf = bytearray()
        iter_addr = address
        # there's a more efficient way to do this, but i want something i know
        # is correct
        for i in range(0, length):
            line_addr = self.byte_addr_to_line_addr(iter_addr)
            block_addr = iter_addr & block_addr_mask

            byte = self.mem[line_addr][block_addr]

            res_buf.append(byte)

            iter_addr += 1

        return res_buf

    def line_addr_to_byte_addr(self, line_addr):
        block_addr_width = (self.mem_bytes - 1).bit_length()

        byte_addr = line_addr << block_addr_width
        return byte_addr
    
    def byte_addr_to_line_addr(self, byte_address):
        block_addr_width = (self.mem_bytes - 1).bit_length()

        line_addr = byte_address >> block_addr_width
        return line_addr

    
    def write_mem(self, byte_address, data: bytearray):
        block_addr_width = (self.mem_bytes - 1).bit_length()
        block_addr_mask = (1 << block_addr_width) - 1

        iter_addr = byte_address
        for data_byte in data:
            line_addr = self.byte_addr_to_line_addr(iter_addr)
            block_addr = iter_addr & block_addr_mask

            self.mem[line_addr][block_addr] = data_byte
            iter_addr += 1


    async def run_mem(self):
        self.in_bus.rdy.value = 1
        while True:
            if self.input_delay_gen is None:
                await ReadOnly()

                if (self.in_bus.wr_en.value == 0 and
                        self.in_bus.rd_en.value == 0):
                    await First(RisingEdge(self.in_bus.wr_en),
                                RisingEdge(self.in_bus.rd_en))
                    await ReadOnly()
                
            else:
                self.in_bus.rdy.value = 0
                await ReadOnly()

                if (self.in_bus.wr_en.value == 0 and
                        self.in_bus.rd_en.value == 0):
                    await First(RisingEdge(self.in_bus.wr_en),
                                RisingEdge(self.in_bus.rd_en))

                delay = self.input_delay_gen.get_delay()
                await ClockCycles(self.clk, delay)
                self.in_bus.rdy.value = 1
                await ReadOnly()

            await self.do_outloop()
            if self.input_delay_gen is not None:
                self.in_bus.rdy.value = 0

            # okay now gets tricky wait until read only to check everything
            await ReadOnly()
            if self.out_bus.rd_rdy.value != 1:
                await RisingEdge(self.out_bus.rd_rdy)

            self.log.debug(f"Successfully outputted")
            # by now, we know we're outputting the value. if that's the case,
            # then we can possibly take another operation this cycle if
            # there's no input delay
            if self.input_delay_gen is None:
                if self.in_bus.wr_en.value == 1: 
                    self.log.debug("Taking in write operation")
                    continue
                elif self.in_bus.rd_en.value == 1:
                    self.log.debug("Taking in read operation")
                    continue
                else:
                    self.log.debug("No op")
                    await RisingEdge(self.clk)
                    self.out_bus.rd_val.value = 0
                    self.in_bus.rdy.value = 1
                    self.out_bus.rd_data.value = BinaryValue(value=0, n_bits=self.mem_width)
            else:
                self.log.debug("Delay gen")
                await RisingEdge(self.clk)
                self.out_bus.rd_val.value = 0
                self.out_bus.rd_data.value = BinaryValue(value=0, n_bits=self.mem_width)


    async def do_outloop(self):
        bus_vals = self.in_bus.get_signals()
        bus_vals.addr = bus_vals.addr.integer
        bus_vals.wr_data = bus_vals.wr_data.buff
        if bus_vals.size:
            bus_vals.size = bus_vals.size.integer
        self.log.debug(f"Received memory operation: {bus_vals}")
        await RisingEdge(self.clk)

        mem_op_res = self._do_mem_op(bus_vals)

        self.log.debug(f"Memory operation result: {mem_op_res}")

        if mem_op_res is not None:
            if (len(mem_op_res) < self.mem_bytes):
                # pad the result with zeros if it's not a full line
                mem_op_res += bytes(self.mem_bytes - len(mem_op_res))

            await self._out_mem_op(mem_op_res)
        # it was a write operation, so make sure rd en is actually 0
        else:
            self.out_bus.rd_val.value = 0
            self.out_bus.rd_data.value = BinaryValue(value=0, n_bits=self.mem_width)


    async def _out_mem_op(self, data):
        self.log.debug(f"Outputting memory operation data: {data}")
        if self.op_delay_gen is not None:
            delay = self.op_delay_gen.get_delay()
            await ClockCycles(self.clk, delay)

        self.out_bus.rd_val.value = 1
        if (data is None):
            self.out_bus.rd_data.value = BinaryValue(value=0, n_bits=self.mem_width)
        else:
            self.out_bus.rd_data.value = BinaryValue(value=bytes(data), n_bits=self.mem_width)


    def _do_mem_op(self, bus_vals):
        self.log.debug("Doing memory operation")
        # if we're doing a read
        op_addr = bus_vals.addr
        if (self.use_byte_addr):
            if bus_vals.rd_en == 1:
                mem_buf = self.read_mem(op_addr, bus_vals.size)
                return mem_buf
            else:
                self.write_mem(op_addr, bus_vals.wr_data)
        else:
            if bus_vals.rd_en == 1:
                mem_line = self.mem[op_addr]
                return bytes(mem_line)
            else:
                wr_mask = self.in_bus.byte_en.value.binstr
                wr_data = self.in_bus.wr_data.value.buff
                for i in range(0, self.mem_bytes):
                    if wr_mask[i] == "1":
                        self.mem[op_addr][i] = wr_data[i]

class MemDelayGen(ABC):
    @abstractmethod
    def get_delay(self):
        pass

class RandomDelay(MemDelayGen):
    def __init__(self, seed, min_delay, max_delay):
        self.rand_gen = Random()
        self.rand_gen.seed(seed)
        self.max_delay = max_delay
        self.min_delay = min_delay

    def get_delay(self):
        return self.rand_gen.randint(self.min_delay, self.max_delay)

class ConstDelay(MemDelayGen):
    def __init__(self, delay):
        self.delay = delay

    def get_delay(self):
        return self.delay
